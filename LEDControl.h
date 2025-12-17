#include <Adafruit_NeoPixel.h>
#include <array>

#ifndef LEDCONTROL_H
#define LEDCONTROL_H

#include <esp32-hal-ledc.h>
#include "constants.h"

constexpr neoPixelType COLOR_ORDER = NEO_GRB + NEO_KHZ800;
extern Adafruit_NeoPixel strip;

// 32-bit color constants (RRGGBB packed in 0x00RRGGBB)
constexpr uint32_t COLOR_VALUE_RED    = 0x00FF0000;
constexpr uint32_t COLOR_VALUE_GREEN  = 0x0000FF00;
constexpr uint32_t COLOR_VALUE_BLUE   = 0x000000FF;
constexpr uint32_t COLOR_VALUE_WHITE  = 0x00FFFFFF;
constexpr uint32_t COLOR_VALUE_YELLOW = 0x00FFFF00;
constexpr uint32_t COLOR_VALUE_ORANGE = 0x00FFA500; // R=255,G=165,B=0
constexpr uint32_t COLOR_VALUE_AMBER  = 0x00FFBF00; // R=255,G=191,B=0

// Scoped enum avoids name collisions
enum class LedColor : uint8_t {
  RED = 0,
  GREEN,
  BLUE,
  WHITE,
  YELLOW,
  ORANGE,
  AMBER,
  COUNT
};

// color lookup table (index matches LedColor)
constexpr std::array<uint32_t, static_cast<size_t>(LedColor::COUNT)> COLOR_TABLE = {
  COLOR_VALUE_RED,
  COLOR_VALUE_GREEN,
  COLOR_VALUE_BLUE,
  COLOR_VALUE_WHITE,
  COLOR_VALUE_YELLOW,
  COLOR_VALUE_ORANGE,
  COLOR_VALUE_AMBER
};

// helper to get 32-bit color from LedColor
static inline uint32_t colorValue(LedColor c) {
  return COLOR_TABLE[static_cast<size_t>(c)];
}

enum class LedState : uint8_t {
  OFF,
  ON
};

class LEDControl {
public:
  void begin(uint32_t brightness);

  void ledOn(uint16_t i, LedColor color = LedColor::GREEN);
  void ledOff(uint16_t i);
  // void fadeLedIn(uint16_t i, unsigned long duration);
  // void fadeLedOut(uint16_t i, unsigned long duration);
  void updateLedState(int16_t i = -1);

  LedState getLedState(uint16_t i) const;
  LedColor getLedColor(uint16_t i) const;

private:
  uint32_t ledFadeStartTime = 0;
  uint32_t ledFadeDuration = 0;
  uint32_t maxBrightness = 255;
  LedState ledState[NUM_PIXELS] = { LedState::OFF, LedState::OFF, LedState::OFF, LedState::OFF, LedState::OFF, LedState::OFF, LedState::OFF, LedState::OFF, LedState::OFF, LedState::OFF };
  LedColor ledColor[NUM_PIXELS] = { LedColor::RED, LedColor::RED, LedColor::RED, LedColor::RED, LedColor::RED, LedColor::RED, LedColor::RED, LedColor::RED, LedColor::RED, LedColor::RED };
};

extern LEDControl ledCtrl;

#endif  // LEDCONTROL_H
