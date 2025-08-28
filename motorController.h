#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <Arduino.h>
#include "constants.h"

// === Motor States ===
enum MotorState {
  MOTOR_STOP,
  MOTOR_START_DEPLOY,
  MOTOR_START_RETRACT,
  MOTOR_RUNNING,
  MOTOR_STOPPING
};

enum MotorInterlock {
  MOTOR_NOT_BLOCKED = 0,
  MOTOR_BLOCKED = 1
};


class MotorController {
public:


  void begin(uint8_t en, uint8_t r, uint8_t l,
             uint16_t deployPwm_, uint16_t retractPwm_,
             unsigned long deployDur, unsigned long retractDur);

  void update();
  bool setState(MotorState value);
  MotorState getState() const;

private:
  uint8_t enablePin;
  uint8_t rpwmPin;
  uint8_t lpwmPin;

  uint16_t deployPwm;
  uint16_t retractPwm;
  unsigned long deployDuration;
  unsigned long retractDuration;

  MotorState state = MOTOR_STOP;
  MotorState commandState = MOTOR_STOP;
  MotorInterlock interlockState = MOTOR_NOT_BLOCKED;

  unsigned long startTime = 0;

  void enable(bool on);
  void driveCW();
  void driveCCW();
  void stop();
  void setupHighFreqPWM();
  void triggerAction(int16_t value);
};

extern MotorController actuator;

#endif  // MOTOR_CONTROLLER_H