#include "Arduino.h"
#include "ELatchController.h"
//#include "DebugLogger.h"

/*==== Class Initialization ===*/
ELatchController elatch;

void ELatchController::begin(HBridge* bridge, uint8_t pinSw) {
  hbridge = bridge;
  state = ELATCH_INIT;
  lastState = state;
  enabled = true;
  activationAllowed = true;
  stateStartTime = millis();
  //retriggerBlockUntil = 0;

  latchSwitchPin = pinSw;
  pinMode(latchSwitchPin, INPUT_PULLUP);

  eLatchSwitch = Debounce(pinSw, LOW);  // Initialize debounce with switch pin

  // Reset all attempt counters
  Nb_Attempts_UNLOCK = 0;
  Nb_Attempts_OPEN = 0;
  Nb_Attempts_LOCK = 0;

  currentSwState = digitalRead(latchSwitchPin);
  lastSwState = currentSwState;
  Serial.println(String(F("Switch status INIT: ")) + String(currentSwState));
  risingSwEdge = false;
  fallingSwEdge = false;



  if ((hbridge) && (hbridge->setState(HBRIDGE_STOP)))
    Serial.println(F("ELatchController initialized (HBridge mode STOPPED)"));
}

void ELatchController::updateSwitch(void) {
  bool rawState = digitalRead(latchSwitchPin);
  risingSwEdge = (rawState && !currentSwState);
  fallingSwEdge = (!rawState && currentSwState);
  lastSwState = currentSwState;
  currentSwState = rawState;
  if (lastSwState != currentSwState) {
    Serial.println(String(F("Switch status changed from ")) + String(lastSwState) + String(F(" to ")) + String(currentSwState));
  }
  elatch.switchStatus = currentSwState;

  // eLatchSwitch.update();

  // if (isSwitchHigh()) {
  //   elatch.switchStatus = isSwitchHigh();
  //   // Serial.println(F("Switch status: High"));
  // } else if (isSwitchLow()) {
  //   elatch.switchStatus = isSwitchLow();
  //   // Serial.println(F("Switch status: Low"));
  // }

  // if (elatch.lateSwitchStatus != elatch.switchStatus) {
  //   Serial.println(String(F("Switch status changed from ")) + String(elatch.lateSwitchStatus) + String(F(" to ")) + String(elatch.switchStatus));
  //   elatch.lateSwitchStatus = elatch.switchStatus;
  // }
}

void ELatchController::update() {
  if (!enabled || !activationAllowed) {
    if ((hbridge) && (hbridge->setState(HBRIDGE_STOP)))
      state = ELATCH_IDLE;
    return;
  }

  if (state != lastState) {
    switch (state) {
      case ELATCH_INIT:
        Serial.println(F("E-Latch: INIT"));
        break;
      case ELATCH_LOCK:
        Serial.println(F("E-Latch: LOCK"));
        break;
      case ELATCH_UNLOCK: Serial.println(F("E-Latch: UNLOCK")); break;
      case ELATCH_OPEN: Serial.println(F("E-Latch: OPEN")); break;
      case ELATCH_IDLE: Serial.println(F("E-Latch: IDLE")); break;
    }
    PrevState = lastState;
    lastState = state;
    stateStartTime = millis();
    // Added by Kiran for testing
    //handleState();
  }
  // to refresh the state
  handleState();

  if (hbridge) {  // refresh the motor state
    hbridge->update();
  }
}


void ELatchController::handleState() {
  uint32_t now = millis();

  updateSwitch();


  switch (state) {
    case ELATCH_INIT:
      //Serial.println(F("ELATCH init"));
      // Change to STOP only if not already set to STOP.
      if ((hbridge) && hbridge->getState() != HBRIDGE_STOP) {
        if (hbridge->setState(HBRIDGE_STOP))
          Serial.println(F("ELATCH_INIT Hbridge state changed to STOP: "));
      }
      Serial.println(String(F("ELATCH_INIT switch state before trigger: ")) + String(elatch.switchStatus));
      if (1 == elatch.switchStatus) {
        // If the switch is High, we assume it's locked and proceed to lock
        transitionTo(ELATCH_LOCK);  // Transition to lock state on init
        // Reset all attempt counters
        Nb_Attempts_UNLOCK = 0;
        Nb_Attempts_OPEN = 0;
        Nb_Attempts_LOCK = 0;
      }

      break;

    case ELATCH_LOCK:

      // Reset the attempts for Unlock and Open attempts
      Nb_Attempts_UNLOCK = 0;
      Nb_Attempts_OPEN = 0;
      // Process the lock only if it is from UNLOCK
      if (PrevState == ELATCH_UNLOCK) {
        // Wait for the timeout for relaxing the E latch for locking
        if (0 == elatch.switchStatus) {
          // lock as the lock timer expired...
          if (now - stateStartTime >= ELATCH_LOCK_TIMEOUT_MS) {
            // Check if it is not manually locked
            if (hbridge->getState() == HBRIDGE_STOP) {
              // Check for attempts
              if (Nb_Attempts_LOCK < 3) {
                if (hbridge->setState(HBRIDGE_CCW)) {
                  Nb_Attempts_LOCK++;  // increment the attempt counter only if the set state is successful
                  Serial.println(String(F("ELATCH_LOCK: Lock Attempt: ")) + String(Nb_Attempts_LOCK));
                }
              }  // Check for attempts

            } /*else
              Serial.println(String(F("ELATCH_LOCK: Waiting for Hbridge to STOP, current state: ")) + String(hbridge->getState()));*/
            stateStartTime = millis();  //for stopping time
            Serial.println(F("ELATCH_LOCK: Timer Stopped"));
          }  // lock as the lock timer expired...

        }  // Wait for the timeout for relaxing the E latch for locking
        // else {
        //   Serial.println(F("ELATCH_LOCK: Switch is High!!!"));
        // }
      }
      break;

    case ELATCH_UNLOCK:
      //Serial.println(F("ELATCH Unlock"));
      // If the switch is Low, we assume it's already unlock or open
      if (1 == elatch.switchStatus && (Nb_Attempts_UNLOCK < MAX_ATTEMPTS)) {
        //Serial.println(String("ELATCH_UNLOCK hbridge state before trigger: ") + String(hbridge->getState()));
        // If the switch is High(locked), we proceed with unlocking
        if (hbridge->getState() == HBRIDGE_STOP) {
          if (hbridge->setState(HBRIDGE_CW)) {
            hbridge->setCwTimeout(80);
            Nb_Attempts_UNLOCK++;  // increment the attempt counter
            Serial.println(String(F("ELATCH_UNLOCK: Unlock Attempt: ")) + String(Nb_Attempts_UNLOCK));
          }
          //Serial.println(String("ELATCH_UNLOCK hbridge state after trigger: ") + String(hbridge->getState()));
          if (!Nb_Attempts_LOCK) Nb_Attempts_LOCK = 0;  // reset so that next lock can happen only 3 times
        } /*else {
          Serial.println(String(F("ELATCH_UNLOCK: Waiting for Hbridge to STOP, current state: ")) + String(hbridge->getState()));
        }*/
      } else
        Serial.println(F("ELATCH_UNLOCK: Switch is LOW!!!"));
      break;

    case ELATCH_OPEN:

      //Serial.println(F("ELATCH open"));
      // If the switch is Low, we assume it's already unlocked
      if (0 == elatch.switchStatus) {
        // If the switch is Low and Unlocked, we proceed with Open
        if (hbridge->getState() == HBRIDGE_STOP) {
          if (Nb_Attempts_OPEN < MAX_ATTEMPTS) {

            if (hbridge->setState(HBRIDGE_CW)) {
              hbridge->setCwTimeout(50);
              Nb_Attempts_OPEN++;  // increment the attempt counter
              Serial.println(String(F("ELATCH_OPEN: Open Attempt: ")) + String(Nb_Attempts_OPEN));
            }
            if (!Nb_Attempts_LOCK) Nb_Attempts_LOCK = 0;  // reset so that next lock can happen only 3 times
          }                                               /*else
            Serial.println(F("ELATCH_OPEN:  Attempts over!!!"));*/
        } else
          Serial.println(F("ELATCH_OPEN: Waiting for Hbridge to STOP, current state: ")) + String(hbridge->getState());
      } else
        Serial.println(F("ELATCH_OPEN: Waiting for UNLOCK to finish."));
      break;

    case ELATCH_IDLE:
      /*             if (now < retriggerBlockUntil) {
                // Block triggers
                
            } */
      break;
  }
}

void ELatchController::lock() {
  // if (!enabled || !activationAllowed) return;
  transitionTo(ELATCH_LOCK);
}
void ELatchController::unlock() {

  if (state == ELATCH_LOCK && elatch.switchStatus) {
    // if (!enabled || !activationAllowed) return;
    transitionTo(ELATCH_UNLOCK);
  }
}
void ELatchController::open() {
  // if (!enabled || !activationAllowed) return;
  if (state == ELATCH_UNLOCK)
    transitionTo(ELATCH_OPEN);
  else
    Serial.println(F("ELATCH open should be after unlock"));
  ;
}
void ELatchController::setIdle() {
  transitionTo(ELATCH_IDLE);
}

void ELatchController::transitionTo(ELatchState newState) {
  state = newState;
  Serial.println(String(F("ELATCH state transition to: ")) + String(newState));
  // stateStartTime set in update()
}
ELatchState ELatchController::getState() const {
  return state;
}

void ELatchController::enableActivation(bool en) {
  activationAllowed = en;
}
// void ELatchController::enable(bool en) { enabled = en; }
bool ELatchController::isEnabled() const {
  return enabled;
}




bool ELatchController::isSwitchRising(void) {
  return eLatchSwitch.isReleased();
}

bool ELatchController::isSwitchFalling(void) {
  return eLatchSwitch.isPressed();
}

bool ELatchController::isSwitchHigh(void) {
  return eLatchSwitch.isUp();
}

bool ELatchController::isSwitchLow(void) {
  return eLatchSwitch.isDown();
}
