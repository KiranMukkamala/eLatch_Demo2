#include <Arduino.h>
#include "ADCReader.h"
#include <math.h>

/*==== Class Initialization ===*/
ADCReader userPotiDeploy;

ADCReader::ADCReader() {}

// platform ADC max
#if defined(ARDUINO_ARCH_ESP32)
  static const uint32_t ADC_MAX = 4095u;
#else
  static const uint32_t ADC_MAX = 1023u;
#endif

// file-scope published voltage to track last output value and avoid jitter updates
static float publishedVoltage = -1.0f;

void ADCReader::begin(uint8_t pin, uint8_t numSamples, float refVoltage) {
  _pin = pin;
  _numSamples = numSamples ? numSamples : 1;
  _refVoltage = refVoltage;

  pinMode(_pin, INPUT);

#if defined(ARDUINO_ARCH_ESP32)
  // typical full-scale for pot on 3.3V
  analogSetPinAttenuation(_pin, ADC_11db);
  analogReadResolution(12);
#endif

  // initialize values
  int r = analogRead(_pin);
  _lastRaw = r;
  _lastAverage = (uint32_t)r;
  _lastVoltage = ((float)_lastAverage / (float)ADC_MAX) * _refVoltage;
  _sampleSum = 0;
  _sampleIndex = 0;
  _newAverageAvailable = false;

  // init EMA/debounce internals
  _filteredRaw = (float)r;
  _filteredInit = true;
  _lastOutputRaw = (uint16_t)r;
  _stableCounter = 0;

  // initialize published voltage to avoid a first-change glitch
  publishedVoltage = _lastVoltage;
}

void ADCReader::update() {
  // Use small oversample + EMA + stable-count debounce
  const int SAMPLES = 4; // small oversample to reduce single-read jitter
  uint32_t sum = 0;
  for (int i = 0; i < SAMPLES; ++i) {
    sum += (uint32_t)analogRead(_pin);
    delayMicroseconds(5);
  }
  uint16_t raw = (uint16_t)(sum / SAMPLES);
  _lastRaw = raw;

  if (!_filteredInit) {
    _filteredRaw = (float)raw;
    _filteredInit = true;
  }

  // EMA update
  _filteredRaw = _filteredRaw + _filterAlpha * ((float)raw - _filteredRaw);

  // Compare rounded filtered value against last published raw
  int filteredRounded = (int)roundf(_filteredRaw);
  int lastOut = (int)_lastOutputRaw;
  if (abs(filteredRounded - lastOut) > (int)_changeThreshold) {
    _stableCounter++;
    if (_stableCounter >= _stableCount) {
      _stableCounter = 0;
      _lastOutputRaw = (uint16_t)filteredRounded;

      // Update public averages/voltage and mark as new
      _lastAverage = (uint32_t)_lastOutputRaw;
      _lastVoltage = ((float)_lastAverage / (float)ADC_MAX) * _refVoltage;
      _newAverageAvailable = true;

      // update publishedVoltage for compatibility with prior logic
      publishedVoltage = _lastVoltage;
    } else {
      _newAverageAvailable = false;
    }
  } else {
    // not yet significant/stable
    _stableCounter = 0;
    _newAverageAvailable = false;
  }
}

int ADCReader::getRaw() const {
  return _lastRaw;
}

uint32_t ADCReader::getAverage() const {
  return _lastAverage;
}

float ADCReader::getVoltage() const {
  return _lastVoltage;
}

uint32_t ADCReader::getScaled(float inMin, float inMax, float outMin, float outMax) const {
  if (inMax <= inMin) return (uint32_t)outMin;

  float inVal = (float)_lastAverage;
  if (inVal < inMin) inVal = inMin;
  if (inVal > inMax) inVal = inMax;

  float ratio = (inVal - inMin) / (inMax - inMin);
  float outVal = outMin + ratio * (outMax - outMin);

  if (outVal < outMin) outVal = outMin;
  if (outVal > outMax) outVal = outMax;

  return (uint32_t)(outVal + 0.5f);
}

bool ADCReader::hasNewAverage() const {
  return _newAverageAvailable;
}

void ADCReader::setNewAverage(bool treated){
  _newAverageAvailable = treated;
}