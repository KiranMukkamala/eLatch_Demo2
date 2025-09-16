#include "DoorHandleController.h"

DoorHandleController::DoorHandleController()
  : doorHandleState(DOOR_HANDLE_INIT),
    buttonDeploy(nullptr), buttonRetract(nullptr), buttonHandleDeploy(nullptr),
    extcapaSensor(nullptr), inrcapaSensor(nullptr),
    ledCtrl(nullptr), actuator(nullptr), eLatchMotorDriver(nullptr),
    Disable_Locking(false), Nb_Open_Attempt(0) {}

void DoorHandleController::setDependencies(Debounce* deployBtn, Debounce* retractBtn, Debounce* deployHandleBtn,
                                           CapaTouchSensor* extCapa, CapaTouchSensor* inrCapa,
                                           LEDControl* ledCtrl,
                                           MotorController* actuator,
                                           RelayController* eLatchMotorDriver) {
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
          ledCtrl->ledOn();
          // Serial.println(F("Entering DOOR_HANDLE_CLOSED"));
        } else {
          // Serial.println(F("Please close the DOOR!!!"));
          ledCtrl->ledOff();
        }
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
  if ((!Disable_Locking) && inrcapaSensor && inrcapaSensor->getCurrentState()) {
    if (extcapaSensor) extcapaSensor->enable(false);
    Disable_Locking = true;
  } else if (Disable_Locking && inrcapaSensor && (!inrcapaSensor->getCurrentState())) {
    if (extcapaSensor) extcapaSensor->enable(true);
    Disable_Locking = false;
  }
  // Capa sensor update
  if (inrcapaSensor) inrcapaSensor->update();
  if (extcapaSensor) extcapaSensor->update();
}

void DoorHandleController::refreshState() {
  // refresh the state machine
  switch (doorHandleState) {
    case DOOR_HANDLE_INIT:
      if (latchSwitchState && actuator->setState(MOTOR_START_RETRACT) && eLatchMotorDriver->setState(RELAY_CCW))
        setState(DOOR_HANDLE_CLOSED);
      // else
      // Serial.println(F("DOOR_HANDLE_INIT:: Waiting for DOOR close status"));
      inrcapaSensor->enable(false);
      inrcapaSensor->enable(true);
      extcapaSensor->enable(false);
      extcapaSensor->enable(true);
      break;

    case DOOR_HANDLE_CLOSED:
      // Process Deploy switch event, check for MOTOR status then move to next state.
      // if (buttonDeploy.isPressed() && (actuator.getState() == MOTOR_STOP)) {
      if ((buttonDeploy->getswitchStatus() || buttonHandleDeploy->getswitchStatus()) && (actuator->getState() == MOTOR_STOP)) {
        setState(DOOR_HANDLE_DEPLOYED);
      }
      // else {
      //   // buttonDeploy.update();
      //   // Serial.println("DOOR_HANDLE_CLOSED:: Waiting for Trigger :: " + String(buttonDeploy.getswitchStatus()) + " " + String(actuator.getState()));
      // }
      if (latchSwitchState)
        ledCtrl->ledOn();
      break;

    case DOOR_HANDLE_RETRACT:

      if (actuator->setState(MOTOR_START_RETRACT) && eLatchMotorDriver->setState(RELAY_CCW)) {
        // Serial.println(F("DOOR_HANDLE_RETRACT:: CCW Triggered"));
        inrcapaSensor->enable(false);
        extcapaSensor->enable(false);
        setState(DOOR_HANDLE_CLOSED);
      }
      // else {
      //   // Serial.println(F("DOOR_HANDLE_RETRACT:: Waiting for Actuator & elatch status!!!"));
      // }
      break;

    case DOOR_HANDLE_DEPLOYED:

      // actuator.triggerAction(4000);
      // go to next state only if the actuator and eLatch are deployed and ready
      if (actuator->setState(MOTOR_START_DEPLOY) && eLatchMotorDriver->setState(RELAY_CW)) {
        // Serial.println(F("DOOR_HANDLE_DEPLOYED:: CW Triggered"));
        delay(200);
        inrcapaSensor->enable(false);
        inrcapaSensor->enable(true);
        extcapaSensor->enable(false);
        extcapaSensor->enable(true);
        setState(DOOR_HANDLE_WAIT_OPEN);
      } else {
        Check_Disable_Locking();
        // Serial.println(F("DOOR_HANDLE_DEPLOYED:: Waiting for Actuator & elatch status!!!"));
      }
      break;

    case DOOR_HANDLE_WAIT_OPEN:
      this->Check_Disable_Locking();

      // Incase of Retract switch pressed or external lock capa sensor pressed, retract the handle
      if (latchSwitchState && (buttonRetract->getswitchStatus() || extcapaSensor->getCurrentState())) {
        setState(DOOR_HANDLE_RETRACT);
      } else {  // just fade in and fade out LED
        if (ledCtrl->getState() == LED_ON)
          ledCtrl->fadeLedIn(LED_FADE_IN_TIME_MS);
        else if (ledCtrl->getState() == LED_OFF)
          ledCtrl->fadeLedOut(LED_FADE_OUT_TIME_MS);
      }
      break;

    case DOOR_HANDLE_OPEN:
      if (latchSwitchState) {
        if (Nb_Open_Attempt <= NB_OPEN_RETRY_COUNT) {
          if ((eLatchMotorDriver->getState() == RELAY_STOP) && eLatchMotorDriver->setState(RELAY_CW)) {
            if (ledCtrl->getState() == LED_OFF)
              ledCtrl->ledOn();
            ++Nb_Open_Attempt;
            // Serial.println(F("DOOR_HANDLE_OPEN:: Waiting for eLatch to open!!!"));
          }
        }
      } else if ((eLatchMotorDriver->getRecentRun() == RELAY_CW) && (eLatchMotorDriver->getState() == RELAY_STOP)) {
        if (ledCtrl->getState() == LED_ON)
          ledCtrl->ledOff();
        Check_Disable_Locking();
        setState(DOOR_HANDLE_WAIT_TO_LATCH);
        // Serial.println(F("DOOR_HANDLE_OPEN:: Waiting for DOOR latched!!!"));
      }
      break;
    case DOOR_HANDLE_WAIT_TO_LATCH:
      inrcapaSensor->enable(false);
      extcapaSensor->enable(false);

      if (latchSwitchState) {
        if (ledCtrl->getState() == LED_OFF)
          ledCtrl->ledOn();
        setState(DOOR_HANDLE_LATCHED);
      } else {
        if (ledCtrl->getState() == LED_ON)
          ledCtrl->ledOff();
      }
      break;
    case DOOR_HANDLE_LATCHED:
      inrcapaSensor->enable(true);
      extcapaSensor->enable(true);
      Check_Disable_Locking();

      // Incase of Retract switch pressed or external lock capa sensor pressed, retract the handle
      if (latchSwitchState && (buttonRetract->getswitchStatus() || (extcapaSensor->getCurrentState() && (!inrcapaSensor->getCurrentState())))) {
        setState(DOOR_HANDLE_RETRACT);
      } else {  // just fade in and fade out LED
        if (ledCtrl->getState() == LED_ON)
          ledCtrl->fadeLedIn(LED_FADE_IN_TIME_MS);
        else if (ledCtrl->getState() == LED_OFF)
          ledCtrl->fadeLedOut(LED_FADE_OUT_TIME_MS);
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
