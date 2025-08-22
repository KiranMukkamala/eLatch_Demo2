#include "LEDControl.h"

void LEDControl::begin(uint8_t ledPin, uint32_t brightness) {
  _ledPin = ledPin;
  maxBrightness = brightness;
  pinMode(_ledPin, OUTPUT);
  Serial.println(F("LED driver initialization completed"));
}

void LEDControl::ledOn() {
  analogWrite(_ledPin, maxBrightness);
  ledState = LED_ON;
}

void LEDControl::ledOff() {
  analogWrite(_ledPin, 0);
  ledState = LED_OFF;
}

void LEDControl::fadeLedIn(unsigned long duration) {
  ledFadeStartTime = millis();
  ledFadeDuration = duration;
  ledState = LED_FADING_IN;
}

void LEDControl::fadeLedOut(unsigned long duration) {
  ledFadeStartTime = millis();
  ledFadeDuration = duration;
  ledState = LED_FADING_OUT;
}

void LEDControl::updateLedState() {
  if (ledState == LED_FADING_IN || ledState == LED_FADING_OUT) {
    uint32_t elapsed = millis() - ledFadeStartTime;

    if (elapsed >= ledFadeDuration) {
      if (ledState == LED_FADING_IN) {
        analogWrite(_ledPin, maxBrightness);
        ledState = LED_ON;
      } else {
        analogWrite(_ledPin, 0);
        ledState = LED_OFF;
      }
      return;
    }

    // Integer math: scaled progress (0 to 1024)
    uint32_t progress = (elapsed * 1024UL) / ledFadeDuration;
    uint32_t brightnessCalc;

    if (ledState == LED_FADING_IN) {
      brightnessCalc = (maxBrightness * progress) / 1024UL;
    } else {
      brightnessCalc = (maxBrightness * (1024UL - progress)) / 1024UL;
    }

    analogWrite(_ledPin, constrain(brightnessCalc, 0, maxBrightness));
  }
}

LedState LEDControl::getState() const {
  return ledState;
}
