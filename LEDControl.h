#ifndef LEDCONTROL_H
#define LEDCONTROL_H

#include <Arduino.h>
#include "constants.h"

enum LedState {
  LED_OFF,
  LED_ON,
  LED_FADING_IN,
  LED_FADING_OUT
};

class LEDControl {
public:
  void begin(uint8_t ledPin, uint32_t brightness);

  void ledOn();
  void ledOff();
  void fadeLedIn(unsigned long duration);
  void fadeLedOut(unsigned long duration);
  void updateLedState();

  LedState getState() const;

private:
  uint32_t ledFadeStartTime = 0;
  uint32_t ledFadeDuration = 0;
  uint8_t _ledPin = 0;
  uint32_t maxBrightness = 255;
  LedState ledState = LED_OFF;
};

extern LEDControl ledCtrl;

#endif // LEDCONTROL_H
