#include <fstream>
#include <stdint.h>
#include "HardwareSerial.h"
#include "esp32-hal.h"
#include "LEDControl.h"

Adafruit_NeoPixel strip(NUM_PIXELS, LED_PWM_PIN, COLOR_ORDER);

void LEDControl::begin(uint32_t brightness) {
  strip.begin();  // Initialize NeoPixel strip
  strip.setBrightness(brightness);
  maxBrightness = brightness;
  
  // Initialize all LEDs as OFF
  for (uint16_t i = 0; i < NUM_PIXELS; ++i) {
    strip.setPixelColor(i, 0);
    ledState[i] = LedState::OFF;
    ledColor[i] = LedColor::RED;
  }
  
  strip.show();   // Turn off all LEDs initially
  // Serial.println("LED driver initialization completed");
}

void LEDControl::ledOn(uint16_t i, LedColor color) {
  if (i >= NUM_PIXELS) return;
  
  uint32_t rgb = colorValue(color);
  strip.setPixelColor(i, rgb);
  ledState[i] = LedState::ON;
  ledColor[i] = color;
  strip.show();
}

void LEDControl::ledOff(uint16_t i) {
  if (i >= NUM_PIXELS) return;
  
  strip.setPixelColor(i, 0);
  ledState[i] = LedState::OFF;
  strip.show();
}

// void LEDControl::fadeLedIn(uint16_t i, unsigned long duration) {
//   if (i >= NUM_PIXELS) return;
  
//   ledFadeStartTime = millis();
//   ledFadeDuration = duration;
//   ledState[i] = LedState::FADING_IN;
// }

// void LEDControl::fadeLedOut(uint16_t i, unsigned long duration) {
//   if (i >= NUM_PIXELS) return;
  
//   ledFadeStartTime = millis();
//   ledFadeDuration = duration;
//   ledState[i] = LedState::FADING_OUT;
// }

void LEDControl::updateLedState(int16_t i) {
  
  LedColor color;
  uint32_t rgb;
  //   LedColor color = getLedColor(idx);
  
  
  //   uint32_t rgb = colorValue(color);
  
  // if (ledState[idx] == LedState::FADING_IN || ledState[idx] == LedState::FADING_OUT) {
  //   uint32_t elapsed = millis() - ledFadeStartTime;

  //   if (elapsed >= ledFadeDuration) {
  //   if (ledState[idx] == LedState::FADING_IN) {
  //     strip.setPixelColor(idx, rgb);
  if (i < 0) {
    for (uint16_t idx = 0; idx < NUM_PIXELS; ++idx) {
      color = getLedColor(idx);
      rgb = colorValue(color);
      if (ledState[idx] == LedState::ON) {
        strip.setPixelColor(idx, rgb);
      } else {
        strip.setPixelColor(idx, 0);
      }
    }

    // Integer math: scaled progress (0 to 1024)
  } else {
    color = getLedColor(i);
    rgb = colorValue(color);
    if (ledState[i] == LedState::ON) {
      strip.setPixelColor(i, rgb);
    } else {
      strip.setPixelColor(i, 0);
    }

    // Apply brightness scaling to the color (simple approach)
    
  }
  
  strip.show();
}

LedState LEDControl::getLedState(uint16_t i) const {
  if (i >= NUM_PIXELS) return LedState::OFF;
  return ledState[i];
}

LedColor LEDControl::getLedColor(uint16_t i) const {
  if (i >= NUM_PIXELS) return LedColor::RED;
  return ledColor[i];
}
