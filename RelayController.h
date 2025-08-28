#ifndef RELAYCONTROLLER_H
#define RELAYCONTROLLER_H

#include <Arduino.h>
#include "constants.h"

enum RelayState {
  RELAY_INIT = 0,
  RELAY_STOP = 1,
  RELAY_CW = 2,      // Clockwise
  RELAY_CCW = 3,     // Counter-clockwise
  RELAY_RUNNING = 4  // Running state, used for timeout
};

enum RelayInterlock {
  RELAY_NON_BLOCKED = 0,
  RELAY_BLOCKED = 1
};


class RelayController {
public:
  void begin(uint8_t r, uint8_t l);

  void update();
  bool setState(RelayState value);
  RelayState getState(void) const;
  RelayState getRecentRun() const;

  // New setters for timeouts and PWM
  void setCwTimeout(uint32_t timeout) {
    cwDuration = timeout;
  }
  void setCcwTimeout(uint32_t timeout) {
    ccwDuration = timeout;
  }

private:
  uint8_t rpwmPin;
  uint8_t lpwmPin;

  // Store timeout per direction
  uint16_t cwDuration = ELATCH_MOTOR_RUN_TIME_CW;
  uint16_t ccwDuration = ELATCH_MOTOR_RUN_TIME_CCW;
  uint16_t stopDuration = ELATCH_MOTOR_STOP_TIME;

  RelayState commandState = RELAY_STOP;
  RelayState lastCommandState = RELAY_INIT;
  RelayInterlock interlockState = RELAY_BLOCKED;

  // NEW: tracks which direction is running in RUNNING state
  RelayState runningDirection = RELAY_STOP;
  RelayState prevrunningDirection = RELAY_STOP;

  uint32_t startTime = 0;

  void driveCW();
  void driveCCW();
  void stop();
};

extern RelayController eLatchMotorDriver;

#endif  // RELAYCONTROLLER_H
