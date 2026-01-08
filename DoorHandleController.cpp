#include "DoorHandleController.h"

DoorHandleController::DoorHandleController()
  : doorHandleState(DOOR_HANDLE_INIT),
    buttonDeploy(nullptr), buttonRetract(nullptr), buttonHandleDeploy(nullptr),
    extcapaSensor(nullptr), inrcapaSensor(nullptr),
    ledCtrl(nullptr), actuator(nullptr), eLatchMotorDriver(nullptr),
    Disable_Locking(false), Nb_Open_Attempt(0) {}

void DoorHandleController::setDependencies(Debounce* deployBtn, Debounce* retractBtn, bool* deployHandleBtn,
                                           uint16_t* extCapa, uint16_t* inrCapa,
                                           LEDControl* ledCtrl,
                                           MotorController* actuator,
                                           MotorController* eLatchMotorDriver) {
  buttonDeploy = deployBtn;
  buttonRetract = retractBtn;
  buttonHandleDeploy = deployHandleBtn;
  extcapaSensor = extCapa;
  inrcapaSensor = inrCapa;
  this->ledCtrl = ledCtrl;
  this->actuator = actuator;
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
          // Serial.println(F("Entering DOOR_HANDLE_CLOSED"));
        } 
        // else {
        //   // Serial.println(F("Please close the DOOR!!!"));
        //   ledCtrl->ledOff(5);
        // }
        break;
      case DOOR_HANDLE_RETRACT:
        if ((doorHandleState == DOOR_HANDLE_LATCHED) || (doorHandleState == DOOR_HANDLE_WAIT_OPEN)) {
          doorHandleState = state;
          // Serial.println(F("Entering DOOR_HANDLE_RETRACT"));
        }
        // else {
        //   // Serial.println(F("Cannot Retract DOOR HANDLE!!!"));
        // }
        break;
      case DOOR_HANDLE_DEPLOYED:
        if (doorHandleState == DOOR_HANDLE_CLOSED) {
          doorHandleState = state;
          // Serial.println(F("Entering DOOR_HANDLE_DEPLOYED"));
        }
        // else {
        //   // Serial.println("Cannot Deploy DOOR HANDLE!!! from " + String(doorHandleState));
        // }
        break;
      case DOOR_HANDLE_WAIT_OPEN:
        if (doorHandleState == DOOR_HANDLE_DEPLOYED) {
          doorHandleState = state;
          // Serial.println(F("Entering DOOR_HANDLE_WAIT_OPEN"));
        }
        // else {
        //   // Serial.println(F("Deployment of DOOR HANDLE not correct!!!"));
        // }
        break;
      case DOOR_HANDLE_OPEN:
        if ((doorHandleState == DOOR_HANDLE_WAIT_OPEN) || (doorHandleState == DOOR_HANDLE_LATCHED)) {
          doorHandleState = state;
          Nb_Open_Attempt = 0;
          // Serial.println(F("Entering DOOR_HANDLE_OPEN"));
        }
        // else {
        //   // Serial.println(F("Opening of DOOR HANDLE is not correct at this moment!!!"));
        // }
        break;
      case DOOR_HANDLE_WAIT_TO_LATCH:
        if (doorHandleState == DOOR_HANDLE_OPEN) {
          doorHandleState = state;
          // Serial.println(F("Entering DOOR_HANDLE_WAIT_TO_LATCH"));
        }
        // else {
        //   // Serial.println(F("Latching of DOOR HANDLE before Open is not correct!!!"));
        // }
        break;
      case DOOR_HANDLE_LATCHED:
        if (doorHandleState == DOOR_HANDLE_WAIT_TO_LATCH) {
          doorHandleState = state;
          // Serial.println(F("Entering DOOR_HANDLE_LATCHED"));
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
  // refresh the state machine
  switch (doorHandleState) {
    case DOOR_HANDLE_INIT:
      if (latchSwitchState) {
        if ((!deploymentSwitchState) && (actuator->getState() != MOTOR_RUNNING)) {
          if ((eLatchMotorDriver->getRecentCommand() != MOTOR_START_RETRACT) && eLatchMotorDriver->setState(MOTOR_START_RETRACT))
            setState(DOOR_HANDLE_CLOSED);
          // else
          //   Serial.println(F("DOOR_HANDLE_INIT:: Waiting for Door handle elatch motor status"));
        } else if ((deploymentSwitchState) && (actuator->getRecentCommand() != MOTOR_START_RETRACT) && (actuator->setState(MOTOR_START_RETRACT)));

        // Serial.println(F("DOOR_HANDLE_INIT:: Waiting for Door handle flush status"));
        // else
        //   Serial.println(F("DOOR_HANDLE_INIT:: Waiting for Door handle flush status else"));
      }
      // else
      //   Serial.println(F("DOOR_HANDLE_INIT:: Waiting for DOOR close status"));
      break;

    case DOOR_HANDLE_CLOSED:
      // Process Deploy switch event, check for MOTOR status then move to next state.
      if ((buttonDeploy->getswitchStatus() || (deploymentSwitchState)) && (actuator->getState() == MOTOR_STOP)) {
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

      if ((!deploymentSwitchState) && (actuator->getState() != MOTOR_RUNNING)) {
        // Serial.println(F("DOOR_HANDLE_RETRACT::Complete"));
        if ((eLatchMotorDriver->getRecentCommand() != MOTOR_START_RETRACT) && eLatchMotorDriver->setState(MOTOR_START_RETRACT))
          setState(DOOR_HANDLE_CLOSED);
      } else if ((actuator->getRecentCommand() != MOTOR_START_RETRACT) && (actuator->setState(MOTOR_START_RETRACT))) {
        // Serial.println(F("DOOR_HANDLE_RETRACT:: CCW Triggered"));
      }
      // else {
      //   // Serial.println(F("DOOR_HANDLE_RETRACT:: Waiting for Actuator & elatch status!!!"));
      // }
      break;

    case DOOR_HANDLE_DEPLOYED:
      // go to next state only if the actuator and eLatch are deployed and ready
      if ((actuator->getRecentCommand() != MOTOR_START_DEPLOY) && actuator->setState(MOTOR_START_DEPLOY)) {
        // if (actuator->setState(MOTOR_START_DEPLOY)) {
        // Serial.println(F("DOOR_HANDLE_DEPLOYED:: CW Triggered"));
      } else if (deploymentSwitchState && (actuator->getState() != MOTOR_RUNNING)) {
        if ((eLatchMotorDriver->getRecentCommand() != MOTOR_START_DEPLOY) && eLatchMotorDriver->setState(MOTOR_START_DEPLOY))
          setState(DOOR_HANDLE_WAIT_OPEN);
        Check_Disable_Locking();
        // Serial.println(F("DOOR_HANDLE_DEPLOYED:: Waiting for Actuator & elatch status!!!"));
      }
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
            // Serial.println(F("DOOR_HANDLE_OPEN:: Waiting for eLatch to open!!!"));
          }
        }
      } else if ((eLatchMotorDriver->getRecentCommand() == MOTOR_START_DEPLOY) && (eLatchMotorDriver->getState() == MOTOR_STOP)) {
        if (ledCtrl->getLedState(5) == LedState::ON)
          ledCtrl->ledOff(5);
        Check_Disable_Locking();
        setState(DOOR_HANDLE_WAIT_TO_LATCH);
        // Serial.println(F("DOOR_HANDLE_OPEN:: Waiting for DOOR latched!!!"));
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

      // Incase of Retract switch pressed or external lock capa sensor pressed, retract the handle
      if (latchSwitchState && (buttonRetract->getswitchStatus() || ((!Disable_Locking) && *extcapaSensor))) {
        setState(DOOR_HANDLE_RETRACT);
      }
      break;

    default:
      // Serial.println(F("Main state machine error"));
      break;
  }
}

void DoorHandleController::updateeLatchSwitch(void) {
  bool rawState = digitalRead(E_LATCH_SW_PIN);
  risingSwEdge = (rawState && !currentSwState);
  fallingSwEdge = (!rawState && currentSwState);
  lastSwState = currentSwState;
  currentSwState = rawState;
  // if (lastSwState != currentSwState) {
  //   Serial.println(String(F("Switch status changed from ")) + String(lastSwState) + String(F(" to ")) + String(currentSwState));
  // }
  latchSwitchState = currentSwState;
}

bool DoorHandleController::getswitchStatus(void) {
  return this->latchSwitchState;
}


void DoorHandleController::updateeDeploymentStatus(void) {
  bool deployrawState = digitalRead(DEPLOY_HANDLE_SW_PIN);
  deployrisingSwEdge = (deployrawState && !deploycurrentSwState);
  deployfallingSwEdge = (!deployrawState && deploycurrentSwState);
  deploylastSwState = deploycurrentSwState;
  deploycurrentSwState = deployrawState;
  // if (deploylastSwState != deploycurrentSwState) {
  //   Serial.println(String(F("Deploy Switch status changed from ")) + String(deploylastSwState) + String(F(" to ")) + String(deploycurrentSwState));
  // }
  deploymentSwitchState = deploycurrentSwState;
}
bool DoorHandleController::getDeploymentStatus(void) {
  return this->deploymentSwitchState;
}