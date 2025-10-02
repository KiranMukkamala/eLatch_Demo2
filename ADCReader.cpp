#include "ADCReader.h"
//#include "DebugLogger.h"

/*==== Class Initialization ===*/
ADCReader userPotiDeploy;


ADCReader::ADCReader() {}

void ADCReader::begin(uint8_t pin, uint8_t numSamples, float refVoltage) {
  _pin = pin;
  _numSamples = numSamples;
  _refVoltage = refVoltage;

  analogReference(DEFAULT);  // Assumes DEFAULT = 5V
  pinMode(_pin, INPUT);

  _sampleSum = 0;
  _sampleIndex = 0;
  _lastRaw = analogRead(_pin);
  Serial.println(F("User MOC thresolds driver initialization completed"));
}

void ADCReader::update() {
  int raw = analogRead(_pin);
  const int threshold = 2;
  _lastRaw = raw;

  _sampleSum += raw;
  _sampleIndex++;

  if (_sampleIndex >= _numSamples) {
    uint32_t newAverage = _sampleSum / _numSamples;
    if (abs((int)newAverage - (int)_lastAverage) > threshold) {
      _lastAverage = newAverage;
      _lastVoltage = (_lastAverage / 1023.0) * _refVoltage;
      _newAverageAvailable = true;
    } else {
      _newAverageAvailable = false;
    }
    _sampleSum = 0;
    _sampleIndex = 0;
  }
}

int ADCReader::getRaw() const {
  return _lastRaw;
}

uint32_t ADCReader::getAverage() const {
  return _lastAverage;
}

uint32_t ADCReader::getVoltage() const {
  return _lastVoltage;
}

uint32_t ADCReader::getScaled(float inMin, float inMax, float outMin, float outMax) const {
  uint32_t clamped = constrain(_lastAverage, inMin, inMax);
  return map(clamped, inMin, inMax, outMin, outMax);
}

bool ADCReader::hasNewAverage() const {
  return _newAverageAvailable;
}

void ADCReader::setNewAverage(bool treated){
  _newAverageAvailable = treated;
}