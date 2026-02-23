#include "BatteryMonitor.h"

#include <Arduino.h>
#include <esp_idf_version.h>

#if ESP_IDF_VERSION_MAJOR < 5
#include <esp_adc_cal.h>
#endif

BatteryMonitor::BatteryMonitor(uint8_t adcPin, float dividerMultiplier, int8_t statusPin)
    : _adcPin(adcPin), _dividerMultiplier(dividerMultiplier), _statusPin(statusPin) {
  if (_statusPin >= 0) {
    // MCP73832 STAT pin is Open Drain, so we need a pullup.
    // It pulls LOW when charging or charge complete. High-Z when
    // standby/shutdown. Note: Some boards might have external pullup. Internal
    // is safer.
    pinMode(_statusPin, INPUT_PULLUP);
  }
}

uint16_t BatteryMonitor::readPercentage() const { return percentageFromMillivolts(readMillivolts()); }

bool BatteryMonitor::isCharging() const {
  if (_statusPin < 0) return false;
  // LOW = Charging or Charge Complete
  // HIGH (Pullup) = Standby / Shutdown / No Power
  return digitalRead(_statusPin) == LOW;
}

uint16_t BatteryMonitor::readMillivolts() const {
#if ESP_IDF_VERSION_MAJOR < 5
  // ESP-IDF 4.x doesn't have analogReadMilliVolts, so we need to do the calibration manually
  const uint16_t raw = analogRead(_adcPin);
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  const uint16_t mv = esp_adc_cal_raw_to_voltage(raw, &adc_chars);
#else
  // ESP-IDF 5.x has analogReadMilliVolts
  const uint16_t mv = analogReadMilliVolts(_adcPin);
#endif

  return static_cast<uint16_t>(mv * _dividerMultiplier);
}

double BatteryMonitor::readVolts() const { return static_cast<double>(readMillivolts()) / 1000.0; }

uint16_t BatteryMonitor::percentageFromMillivolts(uint16_t millivolts) {
  double volts = millivolts / 1000.0;
  // Polynomial derived from LiPo samples
  double y = -144.9390 * volts * volts * volts + 1655.8629 * volts * volts - 6158.8520 * volts + 7501.3202;

  // Clamp to [0,100] and round
  y = max(y, 0.0);
  y = min(y, 100.0);
  y = round(y);
  return static_cast<int>(y);
}
