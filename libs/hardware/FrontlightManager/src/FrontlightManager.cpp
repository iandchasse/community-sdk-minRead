#include <Arduino.h>
#include <Logging.h>

#ifdef FRONTLIGHT_PRESENT

#include <driver/gpio.h>
#include <driver/ledc.h>

#include "FrontlightManager.h"

FrontlightManager::FrontlightManager()
    : brightness(10),
      colorTemp(50),
      hardwareFault(false),
      enabled(false),
      senseArmed(false),
      probeTriggered(false),
      probeTriggeredMs(0) {}

void FrontlightManager::begin() {
  // Install GPIO ISR service required for attachInterruptArg
  gpio_install_isr_service(0);
  // ----- LED_PWR rail (IRLML6402 P-FET gate, active LOW) -----
  // Plain GPIO — PWMing this would destabilise the AP3012 feedback loop.
  // Brightness is controlled entirely through SHDN (PIN_PWM).
  gpio_config_t pwr_cfg = {};
  pwr_cfg.pin_bit_mask = (1ULL << PIN_LED_PWR);
  pwr_cfg.mode = GPIO_MODE_OUTPUT;
  gpio_config(&pwr_cfg);
  gpio_set_level(PIN_LED_PWR, 1);  // Rail off initially

  // ----- LED_SENSE (active LOW = regulation OK) -----
  gpio_config_t sense_cfg = {};
  sense_cfg.pin_bit_mask = (1ULL << PIN_SENSE);
  sense_cfg.mode = GPIO_MODE_INPUT;
  sense_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
  gpio_config(&sense_cfg);
  // In begin(), after sense_cfg setup
  gpio_set_intr_type((gpio_num_t)PIN_SENSE, GPIO_INTR_POSEDGE);  // RISING = fault

  // ----- LEDC Timer 0: RTC8M, 8-bit — SHDN brightness PWM -----
  ledc_timer_config_t t0 = {.speed_mode = SPEED,
                            .duty_resolution = PWM_RES,
                            .timer_num = TIMER0,
                            .freq_hz = PWM_FREQ,
                            .clk_cfg = LEDC_USE_RTC8M_CLK};
  ledc_timer_config(&t0);

  // ----- LEDC Timer 1: RTC8M, 8-bit, 200 Hz — warm/cool blend -----
  ledc_timer_config_t t1 = {.speed_mode = SPEED,
                            .duty_resolution = PWM_RES,
                            .timer_num = TIMER1,
                            .freq_hz = BLEND_FREQ,
                            .clk_cfg = LEDC_USE_RTC8M_CLK};
  ledc_timer_config(&t1);

  ledc_channel_config_t ch_bright = {.gpio_num = PIN_PWM,
                                     .speed_mode = SPEED,
                                     .channel = CH_BRIGHT,
                                     .intr_type = LEDC_INTR_DISABLE,
                                     .timer_sel = TIMER0,
                                     .duty = 0,
                                     .hpoint = 0};
  ledc_channel_config(&ch_bright);

  ledc_channel_config_t ch_warm = {.gpio_num = PIN_WARM,
                                   .speed_mode = SPEED,
                                   .channel = CH_WARM,
                                   .intr_type = LEDC_INTR_DISABLE,
                                   .timer_sel = TIMER1,
                                   .duty = 0,
                                   .hpoint = 0};
  ledc_channel_config(&ch_warm);

  ledc_channel_config_t ch_cool = {.gpio_num = PIN_COOL,
                                   .speed_mode = SPEED,
                                   .channel = CH_COOL,
                                   .intr_type = LEDC_INTR_DISABLE,
                                   .timer_sel = TIMER1,
                                   .duty = 0,
                                   .hpoint = 0};
  ledc_channel_config(&ch_cool);
}

// --------------------------------------------------------------------------

void FrontlightManager::enableRail(bool on) {
  gpio_set_level(PIN_LED_PWR, on ? 0 : 1);  // P-FET active LOW
}

void FrontlightManager::disarmSenseInterrupt() {
  LOG_DBG("Frontlight", "disarm senseArmed=%d", senseArmed);
  senseArmed = false;
  // Unconditional — harmless if not attached, prevents stale interrupts
  gpio_isr_handler_remove((gpio_num_t)PIN_SENSE);
}

void FrontlightManager::armSenseInterrupt() {
  LOG_DBG("Frontlight", "arm senseArmed=%d", senseArmed);
  if (senseArmed) return;
  senseArmed = true;
  gpio_isr_handler_add((gpio_num_t)PIN_SENSE, senseISR, this);
}

// --------------------------------------------------------------------------

void IRAM_ATTR FrontlightManager::probeISR(void* arg) {
  FrontlightManager* self = static_cast<FrontlightManager*>(arg);
  self->probeTriggered = true;
  self->probeTriggeredMs = millis();
}

bool FrontlightManager::probeCircuit() {
  probeTriggered = false;
  probeTriggeredMs = 0;

  enableRail(true);

  attachInterruptArg(digitalPinToInterrupt(PIN_SENSE), probeISR, this, FALLING);

  ledc_set_duty(SPEED, CH_BRIGHT, MAX_DUTY);
  ledc_update_duty(SPEED, CH_BRIGHT);
  ledc_set_duty(SPEED, CH_WARM, MAX_DUTY);
  ledc_update_duty(SPEED, CH_WARM);
  ledc_set_duty(SPEED, CH_COOL, MAX_DUTY);
  ledc_update_duty(SPEED, CH_COOL);

  uint32_t start = millis();
  while (!probeTriggered && (millis() - start) < PROBE_TIMEOUT_MS) {
    delay(1);
  }

  detachInterrupt(digitalPinToInterrupt(PIN_SENSE));

  if (probeTriggered) {
    LOG_DBG("Frontlight", "probe regulation acquired in %lums", probeTriggeredMs - start);
  } else {
    LOG_DBG("Frontlight", "probe timed out after %lums — no regulation", PROBE_TIMEOUT_MS);
    // Only zero and kill rail on failure — on success we leave outputs running
    // so there is no open-circuit gap between probe and updateHardware()
    ledc_set_duty(SPEED, CH_BRIGHT, 0);
    ledc_update_duty(SPEED, CH_BRIGHT);
    ledc_set_duty(SPEED, CH_WARM, 0);
    ledc_update_duty(SPEED, CH_WARM);
    ledc_set_duty(SPEED, CH_COOL, 0);
    ledc_update_duty(SPEED, CH_COOL);
    enableRail(false);
  }

  return probeTriggered;
}

// --------------------------------------------------------------------------

bool FrontlightManager::enable() {
  LOG_DBG("Frontlight", "enable called, brightness=%d", brightness);

  disarmSenseInterrupt();
  hardwareFault = false;
  enabled = false;

  if (brightness < MIN_BRIGHTNESS) brightness = MIN_BRIGHTNESS;

  if (!probeCircuit()) {
    hardwareFault = true;
    return false;
  }

  enabled = true;
  updateHardware();

  delay(50);  // Allow SENSE to settle before arming fault ISR —
              // probe leaves the line in a transitional state and
              // nobody is hot-swapping strips in under 50ms

  armSenseInterrupt();
  return true;
}

void FrontlightManager::disable() {
  LOG_DBG("Frontlight", "disable called");
  disarmSenseInterrupt();
  enabled = false;
  enableRail(false);
  updateHardware();
}

bool FrontlightManager::clearFault() { return enable(); }

// --------------------------------------------------------------------------

void FrontlightManager::setBrightness(uint8_t p) {
  if (p < MIN_BRIGHTNESS) p = MIN_BRIGHTNESS;
  if (p > 100) p = 100;
  brightness = p;
  if (enabled) updateHardware();
}

void FrontlightManager::setColorTemperature(uint8_t p) {
  if (p > 100) p = 100;
  colorTemp = p;
  if (enabled) updateHardware();
}

// --------------------------------------------------------------------------

void FrontlightManager::updateHardware() {
  if (!enabled || hardwareFault) {
    LOG_DBG("Frontlight", "updateHardware zeroing — enabled=%d fault=%d", enabled, hardwareFault);
    ledc_set_duty(SPEED, CH_BRIGHT, 0);
    ledc_update_duty(SPEED, CH_BRIGHT);
    ledc_set_duty(SPEED, CH_WARM, 0);
    ledc_update_duty(SPEED, CH_WARM);
    ledc_set_duty(SPEED, CH_COOL, 0);
    ledc_update_duty(SPEED, CH_COOL);
    return;
  }

  uint32_t brightDuty = (brightness * MAX_DUTY) / 100;
  ledc_set_duty(SPEED, CH_BRIGHT, brightDuty);
  ledc_update_duty(SPEED, CH_BRIGHT);

  // Make-before-break: cool first then warm — boost output never open
  uint32_t warmDuty = (colorTemp * MAX_DUTY) / 100;
  uint32_t coolDuty = ((100 - colorTemp) * MAX_DUTY) / 100;
  ledc_set_duty(SPEED, CH_COOL, coolDuty);
  ledc_update_duty(SPEED, CH_COOL);
  ledc_set_duty(SPEED, CH_WARM, warmDuty);
  ledc_update_duty(SPEED, CH_WARM);
}

// --------------------------------------------------------------------------

void IRAM_ATTR FrontlightManager::senseISR(void* arg) {
  FrontlightManager* self = static_cast<FrontlightManager*>(arg);
  // Serial/Logging inside ISR should be avoided; removed

  // Require SENSE to actually be HIGH at ISR time — filters edge glitches
  if (digitalRead(PIN_SENSE) == LOW) return;

  // Only armed after a successful probe, so any RISING edge is a genuine fault
  self->hardwareFault = true;
  self->enabled = false;
  self->senseArmed = false;

  ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_BRIGHT, 0);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_WARM, 0);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_COOL, 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_BRIGHT);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_WARM);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_COOL);
  // Rail brought down by main loop via clearFault()
}

#endif  // FRONTLIGHT_PRESENT