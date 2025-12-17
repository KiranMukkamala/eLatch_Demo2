#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <Arduino.h>
#include <stdint.h>
#include "constants.h"
#include "esp32-hal-ledc.h"

#ifdef ARDUINO_ARCH_ESP32
#include "driver/ledc.h"
#endif

// Optional defaults for ESP32 LEDC (can be overridden in constants.h)
#ifndef MOTOR_PWM_FREQ_HZ
#define MOTOR_PWM_FREQ_HZ 5000      // 2 kHz default PWM frequency
#endif

#ifndef MOTOR_PWM_RES_BITS
#define MOTOR_PWM_RES_BITS 12        // 12-bit resolution (0-255)
#endif

// Default LEDC channels (can be overridden in constants.h)
#ifndef MOTOR_RPWM_LEDC_CHANNEL
#define MOTOR_RPWM_LEDC_CHANNEL 0
#endif
#ifndef MOTOR_LPWM_LEDC_CHANNEL
#define MOTOR_LPWM_LEDC_CHANNEL 1
#endif

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
  // begin: provide enable, right PWM pin, left PWM pin, deploy/retract PWM values and durations (ms).
  void begin(uint8_t en, uint8_t r, uint8_t l,
             uint16_t deployPwm_, uint16_t retractPwm_,
             unsigned long deployDur, unsigned long retractDur);

  void update();
  bool setState(MotorState value);
  MotorState getState() const;
  MotorState getRecentCommand() const;

private:
  uint8_t enablePin = 255;
  uint8_t rpwmPin = 255;
  uint8_t lpwmPin = 255;

#ifdef ARDUINO_ARCH_ESP32
  // ESP32 LEDC related config
  uint8_t rpwmLedcChannel = MOTOR_RPWM_LEDC_CHANNEL;
  uint8_t lpwmLedcChannel = MOTOR_LPWM_LEDC_CHANNEL;
  uint32_t pwmFreqHz = MOTOR_PWM_FREQ_HZ;
  uint8_t pwmResBits = MOTOR_PWM_RES_BITS;
#endif

  uint16_t deployPwm = 0;
  uint16_t retractPwm = 0;
  unsigned long deployDuration = 0;
  unsigned long retractDuration = 0;

  MotorState state = MOTOR_STOP;
  MotorState commandState = MOTOR_STOP;
  MotorState prevCommandDirection = MOTOR_STOP;
  MotorInterlock interlockState = MOTOR_NOT_BLOCKED;

  unsigned long startTime = 0;

  void enable(bool on);
  void driveCW();
  void driveCCW();
  void stop();
  void setupHighFreqPWM();
  void triggerAction(int16_t value);

  // helper: map legacy 0..255 values to configured resolution
  inline uint32_t scaleToResolution(uint32_t v) const {
#ifdef ARDUINO_ARCH_ESP32
    uint32_t maxOut = (1UL << pwmResBits) - 1;
  if (pwmResBits == 8) {
    if (v > 255) v = 255;
    return v;
  }
  if (v <= maxOut) return v;
  uint32_t scaled = (v * maxOut) / 255UL;
  if (scaled > maxOut) scaled = maxOut;
  return scaled;
#else
    return v; // AVR code expects 8-bit values
#endif
  }
};

extern MotorController actuator;

#endif  // MOTORCONTROLLER_H