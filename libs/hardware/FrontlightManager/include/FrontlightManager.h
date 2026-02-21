#pragma once

#ifdef FRONTLIGHT_PRESENT

#include <Arduino.h>
#include <driver/ledc.h>

class FrontlightManager {
 public:
  FrontlightManager();

  void begin();

  // Brightness is 1–100. 0 is not valid — use disable() to turn off.
  void setBrightness(uint8_t percentage);
  void setColorTemperature(uint8_t warmPct);  // 0=all cool, 100=all warm

  // Enable/disable the frontlight. enable() probes the circuit internally.
  // Returns false if the probe failed (no strip, OVP, wiring fault).
  bool enable();
  void disable();

  bool isEnabled() const { return enabled; }
  bool isHardwareFault() const { return hardwareFault; }

  // Re-probe after a fault. Returns false if still faulted.
  bool clearFault();

 private:
  // -------------------------------------------------------
  // PIN DEFINITIONS — adjust these to match your hardware
  // -------------------------------------------------------
  static constexpr gpio_num_t PIN_LED_PWR = GPIO_NUM_17;  // IRLML6402 gate, active LOW
  static constexpr gpio_num_t PIN_PWM = GPIO_NUM_5;       // AP3012 SHDN — brightness PWM
  static constexpr gpio_num_t PIN_WARM = GPIO_NUM_6;      // BSS138 WARM_TOGGLE (W-)
  static constexpr gpio_num_t PIN_COOL = GPIO_NUM_7;      // BSS138 COOL_TOGGLE (C-)
  static constexpr gpio_num_t PIN_SENSE = GPIO_NUM_18;    // LED_SENSE: active LOW = OK
  // -------------------------------------------------------

  // LEDC — Timer 0, RTC8M-clocked, 8-bit — SHDN brightness PWM
  // RTC8M is independent of APB/CPU clock so PWM frequency is stable
  // regardless of dynamic frequency scaling (DFS) power saving modes.
  static constexpr ledc_mode_t SPEED = LEDC_LOW_SPEED_MODE;
  static constexpr ledc_timer_t TIMER0 = LEDC_TIMER_0;
  static constexpr ledc_channel_t CH_BRIGHT = LEDC_CHANNEL_0;
  static constexpr uint32_t PWM_FREQ = 20000;
  static constexpr ledc_timer_bit_t PWM_RES = LEDC_TIMER_8_BIT;
  static constexpr uint32_t MAX_DUTY = (1 << 8) - 1;  // 255

  // LEDC — Timer 1, RTC8M-clocked, 8-bit, 200 Hz — warm/cool blend
  // Both channels share hpoint=0 — overlap is intentional (make-before-break).
  // A brief shared conduction across W-/C- is inconsequential (R37/82Ω limited).
  // An open-circuit boost output is the real hazard: SW flies to 29V → OVP.
  static constexpr ledc_timer_t TIMER1 = LEDC_TIMER_1;
  static constexpr ledc_channel_t CH_WARM = LEDC_CHANNEL_1;
  static constexpr ledc_channel_t CH_COOL = LEDC_CHANNEL_2;
  static constexpr uint32_t BLEND_FREQ = 200;

  static constexpr uint8_t MIN_BRIGHTNESS = 1;
  static constexpr uint32_t PROBE_TIMEOUT_MS = 150;

  bool probeCircuit();
  void enableRail(bool on);
  void armSenseInterrupt();
  void disarmSenseInterrupt();
  void updateHardware();
  static void IRAM_ATTR senseISR(void* arg);
  static void IRAM_ATTR probeISR(void* arg);

  volatile uint8_t brightness;  // always 1–100 when enabled
  volatile uint8_t colorTemp;   // 0–100
  volatile bool hardwareFault;
  volatile bool enabled;
  volatile bool senseArmed;
  volatile bool probeTriggered;
  volatile uint32_t probeTriggeredMs;
};

#endif  // FRONTLIGHT_PRESENT