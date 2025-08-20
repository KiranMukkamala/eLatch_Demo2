#include <stdint.h>
#ifndef ELATCHCONTROLLER_H
#define ELATCHCONTROLLER_H

#include <Arduino.h>
#include "HBridge.h"
#include "Debounce.h"

#define E_LATCH_SW_PIN 12

#define ELATCH_LOCK_TIMEOUT_MS 5000  // ms relaxation for lock detection
//#define ELATCH_UNLOCK_TIMEOUT_MS  800     // ms the latch motor runs to unlock
//#define ELATCH_OPEN_TIMEOUT_MS    1000    // ms for open (if needed)
//#define ELATCH_RETRIGGER_BLOCK_MS 1000    // ms block after operation

#define MAX_ATTEMPTS 2  // Max attempts allowed to protect motor life cycle

enum ELatchState {
  ELATCH_INIT = 0,
  ELATCH_LOCK,
  ELATCH_UNLOCK,
  ELATCH_OPEN,
  ELATCH_IDLE
};

class ELatchController {
public:
  void begin(HBridge* bridge, uint8_t pinSw);
  // Debounce eLatchSwitch(12);

  void update();
  void lock();
  void unlock();
  void open();
  void setIdle();

  ELatchState getState() const;

  void enableActivation(bool en);
  // void enabled(bool en);
  bool isEnabled() const;

private:
  ELatchState state = ELATCH_INIT;
  ELatchState lastState = ELATCH_INIT;
  ELatchState PrevState = ELATCH_INIT;
  HBridge* hbridge = nullptr;
  bool enabled = true;
  bool activationAllowed = true;
  uint32_t stateStartTime = 0;
  //uint32_t retriggerBlockUntil = 0;
  bool switchStatus = false;
  bool lateSwitchStatus = true;

  uint8_t latchSwitchPin;
  // Switch state and edge detection
  Debounce eLatchSwitch = Debounce(0, HIGH);  // Pin will be set in begin()

  // To Store the no.of attempts to CW, CCW
  uint8_t Nb_Attempts_UNLOCK;
  uint8_t Nb_Attempts_OPEN;
  uint8_t Nb_Attempts_LOCK;

  bool risingSwEdge = false;
  bool fallingSwEdge = false;
  bool currentSwState = HIGH;
  bool lastSwState = HIGH;



  void updateSwitch(void);
  bool isSwitchRising(void);
  bool isSwitchFalling(void);
  bool isSwitchHigh(void);
  bool isSwitchLow(void);



  void transitionTo(ELatchState newState);
  void handleState();
};

// Global instance of ELatchController for use throughout the application
extern ELatchController elatch;

#endif
