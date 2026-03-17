#include "DoorHandleController.h"

DoorHandleController::DoorHandleController()
  : doorHandleState(DOOR_HANDLE_INIT),
    buttonDeploy(nullptr), buttonRetract(nullptr),
    extcapaSensor(nullptr), inrcapaSensor(nullptr),
    ledCtrl(nullptr), eLatchMotorDriver(nullptr),
    Disable_Locking(false), Nb_Open_Attempt(0) {}

void DoorHandleController::setDependencies(Debounce* deployBtn, Debounce* retractBtn,
                                           uint16_t* extCapa, uint16_t* inrCapa,
                                           LEDControl* ledCtrl,
                                           MotorController* eLatchMotorDriver) {
  buttonDeploy = deployBtn;
  buttonRetract = retractBtn;
  // buttonHandleDeploy = deployHandleBtn;
  extcapaSensor = extCapa;
  inrcapaSensor = inrCapa;
  this->ledCtrl = ledCtrl;
  this->eLatchMotorDriver = eLatchMotorDriver;
}

DoorHandleState DoorHandleController::getState() const {
  return doorHandleState;
}

void DoorHandleController::setState(DoorHandleState state) {
  if (state != doorHandleState) {
    switch (state) {
      case DOOR_HANDLE_INIT:
        break;
      case DOOR_HANDLE_CLOSED:
        if (latchSwitchState && ((doorHandleState == DOOR_HANDLE_INIT) || (doorHandleState == DOOR_HANDLE_RETRACT))) {
          doorHandleState = state;
          // ledCtrl->ledOn(5, LedColor::RED);
          Serial.println(F("Entering DOOR_HANDLE_CLOSED"));
        }
        // else {
        //   // Serial.println(F("Please close the DOOR!!!"));
        //   ledCtrl->ledOff(5);
        // }
        break;
      case DOOR_HANDLE_RETRACT:
        if ((doorHandleState == DOOR_HANDLE_LATCHED) || (doorHandleState == DOOR_HANDLE_WAIT_OPEN) || (doorHandleState == DOOR_HANDLE_OPEN)) {
          doorHandleState = state;
          Serial.println(F("Entering DOOR_HANDLE_RETRACT"));
        }
        // else {
        //   // Serial.println(F("Cannot Retract DOOR HANDLE!!!"));
        // }
        break;
      case DOOR_HANDLE_DEPLOYED:
        if (doorHandleState == DOOR_HANDLE_CLOSED) {
          doorHandleState = state;
          Serial.println(F("Entering DOOR_HANDLE_DEPLOYED"));
        }
        // else {
        //   // Serial.println("Cannot Deploy DOOR HANDLE!!! from " + String(doorHandleState));
        // }
        break;
      case DOOR_HANDLE_WAIT_OPEN:
        if (doorHandleState == DOOR_HANDLE_DEPLOYED) {
          doorHandleState = state;
          Serial.println(F("Entering DOOR_HANDLE_WAIT_OPEN"));
        }
        // else {
        //   // Serial.println(F("Deployment of DOOR HANDLE not correct!!!"));
        // }
        break;
      case DOOR_HANDLE_OPEN:
        if ((doorHandleState == DOOR_HANDLE_WAIT_OPEN) || ((tempTimerExecuted) && doorHandleState == DOOR_HANDLE_LATCHED)) {
          doorHandleState = state;
          Nb_Open_Attempt = 0;
          startTempTimer = 0;  // reset the timer
          tempTimerExecuted = false;
          Serial.println(F("Entering DOOR_HANDLE_OPEN"));
        }
        // else {
        //   // Serial.println(F("Opening of DOOR HANDLE is not correct at this moment!!!"));
        // }
        break;
      case DOOR_HANDLE_WAIT_TO_LATCH:
        if (doorHandleState == DOOR_HANDLE_OPEN) {
          doorHandleState = state;
          Serial.println(F("Entering DOOR_HANDLE_WAIT_TO_LATCH"));
        }
        // else {
        //   // Serial.println(F("Latching of DOOR HANDLE before Open is not correct!!!"));
        // }
        break;
      case DOOR_HANDLE_LATCHED:
        if (doorHandleState == DOOR_HANDLE_WAIT_TO_LATCH) {
          doorHandleState = state;
          startTempTimer = 0;  // reset the timer
          tempTimerExecuted = false;
          Serial.println(F("Entering DOOR_HANDLE_LATCHED"));
        }
        break;
      default:
        // Serial.println(F("Unknown state!!!"));
        break;
    }
  }
}

void DoorHandleController::Check_Disable_Locking() {
  if ((!Disable_Locking) && inrcapaSensor && *inrcapaSensor) {
    Disable_Locking = true;
  } else if (Disable_Locking && inrcapaSensor && (!(*inrcapaSensor))) {
    Disable_Locking = false;
  }
}

void DoorHandleController::refreshState() {
  updateeLatchSwitch();  //Refresh the switch state every time we try to open the DOOR elatch
  // refresh the state machine
  switch (doorHandleState) {
    case DOOR_HANDLE_INIT:
      if ((latchSwitchState) && (eLatchMotorDriver->getRecentCommand() != MOTOR_START_RETRACT) && eLatchMotorDriver->setState(MOTOR_START_RETRACT)) {
        setState(DOOR_HANDLE_CLOSED);
        // else
        //   Serial.println(F("DOOR_HANDLE_INIT:: Waiting for Door handle elatch motor status"));
      } else
        Serial.println(F("DOOR_HANDLE_INIT:: Waiting for DOOR close status"));
      break;

    case DOOR_HANDLE_CLOSED:
      // Process Deploy switch event, check for MOTOR status then move to next state.
      if (buttonDeploy->getswitchStatus()) {
        setState(DOOR_HANDLE_DEPLOYED);
      }
      // else {
      //   // buttonDeploy.update();
      //   // Serial.println("DOOR_HANDLE_CLOSED:: Waiting for Trigger :: " + String(buttonDeploy.getswitchStatus()) + " " + String(actuator.getState()));
      // }
      // if (latchSwitchState)
      //   ledCtrl->ledOn(5, LedColor::RED);
      break;

    case DOOR_HANDLE_RETRACT:
      // Serial.println(F("DOOR_HANDLE_RETRACT::Complete"));
      if ((eLatchMotorDriver->getRecentCommand() != MOTOR_START_RETRACT) && eLatchMotorDriver->setState(MOTOR_START_RETRACT))
        setState(DOOR_HANDLE_CLOSED);
      // else {
      //   // Serial.println(F("DOOR_HANDLE_RETRACT:: Waiting for elatch status!!!"));
      // }
      break;

    case DOOR_HANDLE_DEPLOYED:
      // go to next state only if the eLatch is ready
      if ((eLatchMotorDriver->getRecentCommand() != MOTOR_START_DEPLOY) && eLatchMotorDriver->setState(MOTOR_START_DEPLOY))
        setState(DOOR_HANDLE_WAIT_OPEN);
      Check_Disable_Locking();
      Serial.println(F("DOOR_HANDLE_DEPLOYED:: Waiting for elatch status!!!"));
      break;

    case DOOR_HANDLE_WAIT_OPEN:
      this->Check_Disable_Locking();

      // Incase of Retract switch pressed or external lock capa sensor pressed, retract the handle
      if (latchSwitchState && (buttonRetract->getswitchStatus() || ((!Disable_Locking) && *extcapaSensor))) {
        setState(DOOR_HANDLE_RETRACT);
      }
      break;

    case DOOR_HANDLE_OPEN:
      if (latchSwitchState) {
        if (Nb_Open_Attempt <= NB_OPEN_RETRY_COUNT) {
          if ((eLatchMotorDriver->getState() == MOTOR_STOP) && eLatchMotorDriver->setState(MOTOR_START_DEPLOY)) {
            ++Nb_Open_Attempt;
            Serial.println(F("DOOR_HANDLE_OPEN:: Waiting for eLatch to open!!!"));
          }
        } else if (buttonRetract->getswitchStatus())  // Reset the Nb Try and retract the handle
        {
          Nb_Open_Attempt = 0;
          setState(DOOR_HANDLE_RETRACT);
        }
      } else if ((eLatchMotorDriver->getRecentCommand() == MOTOR_START_DEPLOY) && (eLatchMotorDriver->getState() == MOTOR_STOP)) {
        if (ledCtrl->getLedState(5) == LedState::ON)
          ledCtrl->ledOff(5);
        Check_Disable_Locking();
        setState(DOOR_HANDLE_WAIT_TO_LATCH);
        Serial.println(F("DOOR_HANDLE_OPEN:: Waiting for DOOR latched!!!"));
      }
      break;
    case DOOR_HANDLE_WAIT_TO_LATCH:
      if (latchSwitchState) {
        // if (ledCtrl->getLedState(5) == LedState::OFF) {
        //   ledCtrl->ledOn(5, LedColor::RED);
        // }
        setState(DOOR_HANDLE_LATCHED);
      }
      // else {

      //   if (ledCtrl->getLedState(5) == LedState::ON)
      //     ledCtrl->ledOff(5);
      // }
      // ledCtrl->updateLedState(5);
      break;
    case DOOR_HANDLE_LATCHED:
      Check_Disable_Locking();

      // Check whether the temporary timer is executed or not
      if ((!startTempTimer) && (eLatchMotorDriver->getRecentCommand() != MOTOR_START_RETRACT) && eLatchMotorDriver->setState(MOTOR_START_RETRACT)) {
        // Start Temporary timer to lock the eLatch and unlock
        startTempTimer = millis();
        Serial.println(F("DOOR_HANDLE_LATCHED:: Temp Timer Started!!!"));
      } else if ((millis() - startTempTimer) >= TempTimerDuration) {  //Timer is expired
        if ((!tempTimerExecuted) && (eLatchMotorDriver->getRecentCommand() != MOTOR_START_DEPLOY) && eLatchMotorDriver->setState(MOTOR_START_DEPLOY)) {
          tempTimerExecuted = true;
          Serial.println(F("DOOR_HANDLE_LATCHED:: Temp Timer Finished!!!"));
        }
        // Incase of Retract switch pressed or external lock capa sensor pressed, retract the handle
        else if (latchSwitchState && (buttonRetract->getswitchStatus() || ((!Disable_Locking) && *extcapaSensor))) {
          setState(DOOR_HANDLE_RETRACT);
          startTempTimer = 0;  // reset the timer
          tempTimerExecuted = true;
          Serial.println(F("DOOR_HANDLE_LATCHED:: Temp Timer Reset!!!"));
        }
      }

      break;

    default:
      Serial.println(F("Main state machine error"));
      break;
  }
}

void DoorHandleController::updateeLatchSwitch(void) {
  bool rawState = digitalRead(E_LATCH_SW_PIN);
  risingSwEdge = (rawState && !currentSwState);
  fallingSwEdge = (!rawState && currentSwState);
  lastSwState = currentSwState;
  currentSwState = rawState;
  latchSwitchState = currentSwState;
}

bool DoorHandleController::getswitchStatus(void) {
  return this->latchSwitchState;
}
