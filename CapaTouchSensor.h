#ifndef CAPATOUCHSENSOR_H
#define CAPATOUCHSENSOR_H

#include <Arduino.h>
#include "constants.h"



class CapaTouchSensor {
public:
  void begin(uint8_t enPin, uint8_t inputPin);
  void update();
  void enable(bool on);

  bool isRising() const;
  bool isFalling() const;
  bool getCurrentState() const;

private:
  uint8_t pin;
  uint8_t enablePin;
  bool lastState = false;
  bool currentState = false;
  bool risingEdge = false;
  bool fallingEdge = false;
};

// extern CapaTouchSensor capaSensor;

#endif  // CAPA_TOUCH_SENSOR_H
