#ifndef ADCREADER_H
#define ADCREADER_H

#include <Arduino.h>

// Lightweight ADC reader for plain averaging. Compatible with ESP32 (12-bit) and AVR (10-bit).
class ADCReader {
public:
  ADCReader();

  // pin: ADC pin number; numSamples: moving average window; refVoltage: full-scale voltage for scaling (e.g. 3.3f)
  void begin(uint8_t pin, uint8_t numSamples, float refVoltage);

  // Call frequently in loop()
  void update();

  int getRaw() const;
  uint32_t getAverage() const;   // raw average ADC units
  float getVoltage() const;      // computed voltage (float)
  uint32_t getScaled(float inMin, float inMax, float outMin, float outMax) const;
  bool hasNewAverage() const;
  void setNewAverage(bool treated);

private:
  uint8_t _pin = 255;           // no implicit pin; must provide in begin()
  uint8_t _numSamples = 10;
  float _refVoltage = 3.3f;

  int _lastRaw = 0;
  uint32_t _lastAverage = 0;    // stored as integer ADC units
  float _lastVoltage = 0.0f;    // computed voltage (float)

  uint8_t _sampleIndex = 0;
  uint32_t _sampleSum = 0;
  bool _newAverageAvailable = false;
  float _filteredRaw = 0.0f;    // internal EMA value (float)
  bool _filteredInit = false;
  uint16_t _lastOutputRaw = 0;  // last published raw value
  uint8_t _stableCounter = 0;
  uint8_t _stableCount = 5;     // STABLE_COUNT (try 4..8)
  uint16_t _changeThreshold = 6; // CHANGE_THRESHOLD (try 4..12 on 12-bit ADC)
  float _filterAlpha = 0.18f;   // FILTER_ALPHA (0.1 = very smooth, 0.18 = smoother)
};

extern ADCReader userPotiDeploy;

#endif  // ADCREADER_H
