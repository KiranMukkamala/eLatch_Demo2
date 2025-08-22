#ifndef HBRIDGE_H
#define HBRIDGE_H

#include <Arduino.h>
#include "constants.h"


enum HBridgeState {
    HBRIDGE_INIT = 0,
    HBRIDGE_STOP = 1,
    HBRIDGE_CW   = 2,   // Clockwise
    HBRIDGE_CCW  = 3,   // Counter-clockwise
    HBRIDGE_RUNNING = 4 // Running state, used for timeout
};

enum HBridgeInterlock{
    HBRIDGE_NON_BLOCKED = 0,
    HBRIDGE_BLOCKED = 1
};


class HBridge {
public:
    void begin(uint8_t en, uint8_t r, uint8_t l);

    void update();
    bool setState(HBridgeState value);
    HBridgeState getState(void) const;

    // New setters for timeouts and PWM
    void setCwTimeout(uint32_t timeout) { cwDuration = timeout; }
    void setCcwTimeout(uint32_t timeout) { ccwDuration = timeout; }
    void setCwPwm(uint16_t pwm) { cwPwm = pwm; }
    void setCcwPwm(uint16_t pwm) { ccwPwm = pwm; }

private:
    uint8_t enablePin;
    uint8_t rpwmPin;
    uint8_t lpwmPin;

    // NEW: Store PWM per direction
    uint16_t cwPwm = DEFAULT_CW_PWM;
    uint16_t ccwPwm = DEFAULT_CCW_PWM;

    // Store timeout per direction
    uint16_t cwDuration = RUN_TIME_CW;
    uint16_t ccwDuration = RUN_TIME_CCW;
    uint16_t stopDuration = STOP_TIME;

    HBridgeState commandState = HBRIDGE_STOP;
    HBridgeState lastCommandState = HBRIDGE_INIT;
    HBridgeInterlock interlockState = HBRIDGE_BLOCKED;


    // NEW: tracks which direction is running in RUNNING state
    HBridgeState runningDirection = HBRIDGE_STOP;

    uint32_t startTime = 0;

    void enable(bool on);
    void driveCW(int16_t value);
    void driveCCW(int16_t value);
    void stop();
    void brake();
    void setupHighFreqPWM();
};

extern HBridge hbridge;

#endif // HBRIDGE_H
