#include "WString.h"
#include "RelayController.h"

/*==== Class Initialization ===*/
RelayController eLatchMotorDriver;

void RelayController::begin(uint8_t r, uint8_t l) {
  rpwmPin = r;
  lpwmPin = l;

  pinMode(rpwmPin, OUTPUT);
  pinMode(lpwmPin, OUTPUT);
  // setupHighFreqPWM();
  stop();

  commandState = RELAY_INIT;
  lastCommandState = commandState;
  interlockState = RELAY_NON_BLOCKED;

  cwDuration = ELATCH_MOTOR_RUN_TIME_CW;
  ccwDuration = ELATCH_MOTOR_RUN_TIME_CW;
  stopDuration = ELATCH_MOTOR_STOP_TIME;

  runningDirection = RELAY_STOP; // set to  CW, CCW or STOP
  prevrunningDirection = RELAY_STOP; // set to  CW, CCW

  Serial.println(F("Elatch Motor driver initialization completed"));
}

bool RelayController::setState(RelayState value) {
  if (interlockState != RELAY_BLOCKED) {
    // Serial.println(String(F("Relay is unblocked, setState changing to ")) + String(value));
    commandState = value;
    return true;
  }
  return false;
}


RelayState RelayController::getState() const {
  return commandState;
}

RelayState RelayController::getRecentRun() const {
  return prevrunningDirection;
}

void RelayController::update() {
  // State machine transitions for checking right transitions allowed
  if (commandState != lastCommandState) {
    // Serial.println(String(F("Relay update commandState changing from ")) + String(lastCommandState) + String(F(" to ")) + String(commandState));
    switch (commandState) {
      case RELAY_INIT:
        if (lastCommandState == RELAY_INIT) {
          // Serial.println(F("Entering RELAY_INIT"));
          lastCommandState = commandState;
        }
        break;

      case RELAY_STOP:
        if (lastCommandState == RELAY_RUNNING || lastCommandState == RELAY_INIT) {
          // Serial.println(F("Relay state changing to STOP"));
          lastCommandState = commandState;
        }
        break;
      case RELAY_CW:
        if (lastCommandState == RELAY_STOP) {
          // Serial.println(F("Relay state changing to CW"));
          lastCommandState = commandState;
        }
        break;
      case RELAY_CCW:
        if (lastCommandState == RELAY_STOP) {
          // Serial.println(F("Relay state changing to CCW"));
          lastCommandState = commandState;
        }
        break;
      case RELAY_RUNNING:
        if (lastCommandState == RELAY_CW || lastCommandState == RELAY_CCW) {
          // Serial.println(F("Relay state changing to RUNNING"));
          lastCommandState = commandState;
        }
        break;
      default:
        Serial.println(F("Unknown Relay state!"));
        break;
    }
  }

  // State machine transitions with actions
  switch (commandState) {
    case RELAY_INIT:
      stop();
      commandState = RELAY_STOP;
      interlockState = RELAY_NON_BLOCKED;
      // Serial.println(F("RELAY_INIT"));
      break;

    case RELAY_STOP:
      // Serial.println(F("RELAY_STOP"));
      stop();
      if (millis() - startTime >= stopDuration) {
        //Serial.println(F("Stopping timeout reached"));
        interlockState = RELAY_NON_BLOCKED;
        runningDirection = RELAY_STOP;  // Clear running direction
      }
      break;

    case RELAY_CW:
      // Serial.println(F("RELAY_CW before stop"));
      // Serial.println(String("State machine state ") + String(getState()));
      // Serial.println(String(F("Relay commandState state: ")) + String(commandState) + String(F(" Relay running direction: ")) + String(runningDirection));
      if (runningDirection == RELAY_STOP) {
        // enable(true);
        driveCW();
        startTime = millis();
        runningDirection = RELAY_CW;  // Remember direction for timeout
        prevrunningDirection = RELAY_CW; // Remember direction for previous run
        commandState = RELAY_RUNNING;
        Serial.println(F("RELAY_CW"));
        interlockState = RELAY_BLOCKED;
      }
      break;

    case RELAY_CCW:
      // Serial.println(F("RELAY_CCW"));
      if (runningDirection == RELAY_STOP) {
        // enable(true);
        driveCCW();
        startTime = millis();
        runningDirection = RELAY_CCW;  // Remember direction for timeout
        prevrunningDirection = RELAY_CCW; // Remember direction for previous run
        commandState = RELAY_RUNNING;
        Serial.println(F("RELAY_CCW"));
        interlockState = RELAY_BLOCKED;
      }
      break;

    case RELAY_RUNNING:
      // Serial.println(F("RELAY_RUNNING"));
      // Serial.println(String("State machine state ") + String(getState()));
      if (runningDirection == RELAY_CW) {
        //Serial.println(String("Time to run ") + String(cwDuration) + String("Start time: ") + String(startTime) + String("Current time") + String(millis()) );
        if (millis() - startTime >= cwDuration) {
          commandState = RELAY_STOP;
          // Serial.println(F("CW timeout reached. Stopping."));
          startTime = millis();  //for stopping time
        }
      } else if (runningDirection == RELAY_CCW) {
        if (millis() - startTime >= ccwDuration) {
          commandState = RELAY_STOP;
          // Serial.println(F("CCW timeout reached. Stopping."));
          startTime = millis();  //for stopping time
        }
      }
      break;

    default:
      Serial.println(F("Something went wrong with Relay state machine"));
      break;
  }  //end switch
}

/*=== Private functions ===*/
void RelayController::driveCCW() {

  digitalWrite(lpwmPin, HIGH);
  digitalWrite(rpwmPin, LOW);
}

void RelayController::driveCW() {

  digitalWrite(lpwmPin, LOW);
  digitalWrite(rpwmPin, HIGH);
}

void RelayController::stop() {

  digitalWrite(rpwmPin, LOW);
  digitalWrite(lpwmPin, LOW);
}

