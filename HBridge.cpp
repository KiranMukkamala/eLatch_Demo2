#include "WString.h"
#include "HBridge.h"
//#include "DebugLogger.h"

/*==== Class Initialization ===*/
HBridge hbridge;

void HBridge::begin(uint8_t en, uint8_t r, uint8_t l) {
  enablePin = en;
  rpwmPin = r;
  lpwmPin = l;

  pinMode(enablePin, OUTPUT);
  pinMode(rpwmPin, OUTPUT);
  pinMode(lpwmPin, OUTPUT);
  digitalWrite(enablePin, LOW);
  setupHighFreqPWM();
  stop();
  enable(false);

  commandState = HBRIDGE_INIT;
  lastCommandState = commandState;
  interlockState = HBRIDGE_NON_BLOCKED;

  cwDuration = RUN_TIME_CW;
  ccwDuration = RUN_TIME_CCW;
  cwPwm = DEFAULT_CW_PWM;
  ccwPwm = DEFAULT_CCW_PWM;
  stopDuration = STOP_TIME;

  runningDirection = HBRIDGE_STOP;

  Serial.println(F("H-Bridge driver initialization completed"));
}

bool HBridge::setState(HBridgeState value) {
  if (interlockState != HBRIDGE_BLOCKED) {
    Serial.println(String(F("HBridge is unblocked, setState changing to ")) + String(value));
    commandState = value;
    return true;
  }
  return false;
}


HBridgeState HBridge::getState() const {
  return commandState;
}

void HBridge::update() {
  // State machine transitions for checking right transitions allowed
  if (commandState != lastCommandState) {
    Serial.println(String(F("HBridge update commandState changing from ")) + String(lastCommandState) + String(F(" to ")) + String(commandState));
    switch (commandState) {
      case HBRIDGE_INIT:
        if (lastCommandState == HBRIDGE_INIT) {
          Serial.println(F("Entering HBRIDGE_INIT"));
          lastCommandState = commandState;
        }
        break;

      case HBRIDGE_STOP:
        if (lastCommandState == HBRIDGE_RUNNING || lastCommandState == HBRIDGE_INIT) {
          Serial.println(F("HBridge state changing to STOP"));
          lastCommandState = commandState;
        }
        break;
      case HBRIDGE_CW:
        if (lastCommandState == HBRIDGE_STOP) {
          Serial.println(F("HBridge state changing to CW"));
          lastCommandState = commandState;
        }
        break;
      case HBRIDGE_CCW:
        if (lastCommandState == HBRIDGE_STOP) {
          Serial.println(F("HBridge state changing to CCW"));
          lastCommandState = commandState;
        }
        break;
      case HBRIDGE_RUNNING:
        if (lastCommandState == HBRIDGE_CW || lastCommandState == HBRIDGE_CCW) {
          Serial.println(F("HBridge state changing to RUNNING"));
          lastCommandState = commandState;
        }
        break;
      default:
        Serial.println(F("Unknown H-Bridge state!"));
        break;
    }
  }

  // State machine transitions with actions
  switch (commandState) {
    case HBRIDGE_INIT:
      stop();
      enable(false);
      commandState = HBRIDGE_STOP;
      interlockState = HBRIDGE_NON_BLOCKED;
      // Serial.println(F("HBRIDGE_INIT"));
      break;

    case HBRIDGE_STOP:
      // Serial.println(F("HBRIDGE_STOP"));
      stop();
      enable(false);

      if (millis() - startTime >= stopDuration) {
        //Serial.println(F("Stopping timeout reached"));
        interlockState = HBRIDGE_NON_BLOCKED;
        runningDirection = HBRIDGE_STOP;  // Clear running direction
      }
      break;

    case HBRIDGE_CW:
      // Serial.println(F("HBRIDGE_CW before stop"));
      // Serial.println(String("State machine state ") + String(getState()));
      // Serial.println(String(F("HBRIDGE commandState state: ")) + String(commandState) + String(F(" HBRIDGE running direction: ")) + String(runningDirection));
      if (runningDirection == HBRIDGE_STOP) {
        // enable(true);
        driveCW(cwPwm);
        startTime = millis();
        runningDirection = HBRIDGE_CW;  // Remember direction for timeout
        commandState = HBRIDGE_RUNNING;
        Serial.println(F("HBRIDGE_CW"));
        interlockState = HBRIDGE_BLOCKED;
      }
      break;

    case HBRIDGE_CCW:
      // Serial.println(F("HBRIDGE_CCW"));
      if (runningDirection == HBRIDGE_STOP) {
        // enable(true);
        driveCCW(ccwPwm);
        startTime = millis();
        runningDirection = HBRIDGE_CCW;  // Remember direction for timeout
        commandState = HBRIDGE_RUNNING;
        Serial.println(F("HBRIDGE_CCW"));
        interlockState = HBRIDGE_BLOCKED;
      }
      break;

    case HBRIDGE_RUNNING:
      Serial.println(F("HBRIDGE_RUNNING"));
      enable(true);
      // Serial.println(String("State machine state ") + String(getState()));
      if (runningDirection == HBRIDGE_CW) {
        //Serial.println(String("Time to run ") + String(cwDuration) + String("Start time: ") + String(startTime) + String("Current time") + String(millis()) );
        if (millis() - startTime >= cwDuration) {
          commandState = HBRIDGE_STOP;
          Serial.println(F("CW timeout reached. Stopping."));
          startTime = millis();  //for stopping time
        }
      } else if (runningDirection == HBRIDGE_CCW) {
        if (millis() - startTime >= ccwDuration) {
          commandState = HBRIDGE_STOP;
          Serial.println(F("CCW timeout reached. Stopping."));
          startTime = millis();  //for stopping time
        }
      }
      break;

    default:
      Serial.println(F("Something went wrong with HBRIDGE state machine"));
      break;
  }  //end switch


}

/*=== Private functions ===*/
void HBridge::enable(bool on) {
  digitalWrite(enablePin, on ? HIGH : LOW);
}

void HBridge::driveCCW(int16_t value) {
  TCCR1A &= ~(1 << COM1B1);
  OCR1B = 0;
  digitalWrite(lpwmPin, LOW);
  OCR1A = value;
  TCCR1A |= (1 << COM1A1);
}

void HBridge::driveCW(int16_t value) {
  TCCR1A &= ~(1 << COM1A1);
  OCR1A = 0;
  digitalWrite(rpwmPin, LOW);
  OCR1B = value;
  TCCR1A |= (1 << COM1B1);
}

void HBridge::stop() {
  TCCR1A &= ~((1 << COM1A1) | (1 << COM1B1));
  OCR1A = 0;
  OCR1B = 0;
  digitalWrite(rpwmPin, LOW);
  digitalWrite(lpwmPin, LOW);
}

void HBridge::brake() {
  TCCR1A &= ~((1 << COM1A1) | (1 << COM1B1));
  OCR1A = 0;
  OCR1B = 0;
  digitalWrite(rpwmPin, HIGH);
  digitalWrite(lpwmPin, HIGH);
}

void HBridge::setupHighFreqPWM() {
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  TCCR1A |= (1 << WGM11) | (1 << WGM10);
  TCCR1B |= (1 << WGM12);
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1);
  TCCR1B |= (1 << CS10);  // No prescaler
}
