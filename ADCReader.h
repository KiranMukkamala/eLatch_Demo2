#ifndef ADCREADER_H
#define ADCREADER_H

#include <Arduino.h>
// #include "global.h"
// #include "constants.h"



class ADCReader {
public:
  ADCReader();

  void begin(uint8_t pin, uint8_t numSamples, float refVoltage);

  void update();  // Call this frequently in loop()

  int getRaw() const;
  uint32_t getAverage() const;
  uint32_t getVoltage() const;
  uint32_t getScaled(float inMin, float inMax, float outMin, float outMax) const;
  bool hasNewAverage() const;
  void setNewAverage(bool treated);

private:
  uint8_t _pin = A0;
  uint8_t _numSamples = 10;
  float _refVoltage = 5.0;

  int _lastRaw = 0;
  float _lastAverage = 0;
  float _lastVoltage = 0;

  uint8_t _sampleIndex = 0;
  long _sampleSum = 0;
  bool _newAverageAvailable = false;
};

extern ADCReader userPotiDeploy;

#endif  // ADC_READER_H
