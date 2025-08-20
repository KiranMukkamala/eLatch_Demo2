#include "CapaTouchSensor.h"
// #include "DebugLogger.h"

CapaTouchSensor capaSensor;

void CapaTouchSensor::begin(uint8_t enPin, uint8_t inputPin) {
  pin = inputPin;
  enablePin = enPin;
  pinMode(pin, INPUT_PULLUP);
  pinMode(enablePin, OUTPUT);
  enable(true);

  currentState = digitalRead(pin);
  lastState = currentState;
  risingEdge = false;
  fallingEdge = false;

   Serial.println(F("Capacitive proximity driver initialization completed"));
}

void CapaTouchSensor::update() {
  currentState = digitalRead(pin);
  risingEdge = (currentState && !lastState);
  fallingEdge = (!currentState && lastState);
  lastState = currentState;
}

void CapaTouchSensor::enable(bool on) {
  digitalWrite(enablePin, on ? HIGH : LOW);
}

bool CapaTouchSensor::isRising() const {
  return risingEdge;
}

bool CapaTouchSensor::isFalling() const {
  return fallingEdge;
}

bool CapaTouchSensor::getCurrentState() const {
  return currentState;
}
