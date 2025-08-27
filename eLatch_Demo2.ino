/**
 * @file main.ino
 * @brief Main application file for AUDi E-Latch Mockup 2.
 * 
 * This sketch reads the capacitive force using MOC technology from Microchip, 
 * detects threshold changes and activate E-Latch when specified threshold is reached.
 * 
 * @details
 * The system uses Serial1 to communicate with the MOC sensor module. 
 * 
 * When a light threshold is detected elatch is unlocked, if the threshold is lower then this threshold elatch goes back to lock mode.
 * From Unlock mode if user increase the pressure on MOC sensor by pulling harder and open thresold is reached the elatch open command is issued.
 * 
 * 
 * 
 * @author
 * Adrian David
 * 
 * @date
 * 2025-08-19
 * 
 * @version
 * 1.3
 *
 * @note
 * Firmware is writen for Arduino Micro board. Debugging is via native Serial port.
 *
 * @bug
 * Starting up is not always corectly completed when debugging via serial is done. For demonstrator debugging must be deactivated.
 * ============================================================================
 *                               PINOUT DIAGRAM
 * ============================================================================
 * 
 *                 +-------------------------+
 *                     Arduino Micro Board     
 *                 +-------------------------+                         
 *                     [TX1] ------> Not Used      
 *                     [RX1] <------ MOC TX       
 *                     [D0]     x                        
 *                     [D1]     x                      
 *                     [D2] ------> EXT CAPA Sensor PWR VDD                 
 *                     [D3] <------ EXT CAPA Sensor OUT                            
 *                     [D4] ------> ELatch Relay       
 *                     [D5]     x
 *                     [D6]     x                            
 *                     [D7]     x
 *                     [D8] ------> H-Bridge Enable
 *                     [D9] ------> H-Bridge RPWM
 *                     [D10]------> H-Bridge LPWM
 *                     [D11]------> 
 *                     [D12]<------ ELatch Switch state
 *                     [A0] ------> Not Used
 *                     [A1] <------ Unlock Threshold                         
 *                     [A2] <------ Open Threshold(Unused)                       
 *                                                   
 *                                                   
 *                                                   
 *                                                   
 *                                               
 * 
 * ============================================================================
 *                          FUNCTIONAL OVERVIEW
 * ============================================================================
 * 
 *  - Initializes MOCReader and ELatchController
 *  - Detects motion/contact based on configured thresholds
 *  - Triggers latch output if change sustained
 *  - Uses a configurable cooldown to suppress false retriggers
 * 
 * ============================================================================
 *
 * 
 * ============================================================================
 * 2025.05.12 - v1.0 First released
 * 2025.05.23 - v1.1 Functional with first mechanical mockup
 * 2025.07.28 - v1.2 TBT with second mechanical mockup 
 * 2025.08.19 - v1.3 Integration with CAPA Sensor
 * ============================================================================
 * 
 */

#include <Arduino.h>
#include "./Debounce.h"

#include "MOCReader.h"
#include "TimeoutManager.h"
// #include "ELatchController.h"
#include "CapaTouchSensor.h"
#include "LEDControl.h"
#include "constants.h"
#include "motorController.h"
#include "RelayController.h"

#define DEBUGGING_ENABLED true
#define SERIAL_DEBUG_SPEED 19200


//=== Manual Control of Door Handle Pin Switch ===
Debounce buttonDeploy(DEPLOY_SW_PIN, LOW);
Debounce buttonRetract(RETRACT_SW_PIN, LOW);
bool switchStatusDeploy = false;
bool switchStatusRetract = false;

// DOOR Handle State
DoorHandleState doorHandleState = DOOR_HANDLE_INIT;
// DoorHandleState lastDoorHandleState = DOOR_HANDLE_INIT;

//CAPA Sensor objects
CapaTouchSensor extcapaSensor;
CapaTouchSensor inrcapaSensor;
bool Disable_Locking = false;  // for disabling the external capa sensor

//LED contzrol object
LEDControl ledCtrl;

//eLatch Switch status
bool latchSwitchState = false;
bool risingSwEdge = false;
bool fallingSwEdge = false;
bool currentSwState = HIGH;
bool lastSwState = HIGH;
uint16_t Nb_Open_Attempt = 0;

void onMOCPull(const MOCSignalData& data) {

  static bool triggerActive = false;
  //static bool unlockTriggerActive = false;

  // Serial.println(String(F("data.diff: ")) + String(abs(data.diff)));
  // Serial.println(String(F("unlockThreshold: ")) + String(data.unlockThreshold));
  // Serial.println(String(F("openThreshold: ")) + String(data.openThreshold));


  //detect user action like pulling
  if (!triggerActive && abs(data.diff) >= data.unlockThreshold) {

    bool monotonicOpen = ((data.mid <= data.oldest) && (data.newest <= data.mid));
    if (monotonicOpen) {
      Serial.println(String(F("data.diff: ")) + String(abs(data.diff)));
      // Serial.println(String(F("unlockThreshold: ")) + String(data.unlockThreshold));
      Serial.println(String(F("openThreshold: ")) + String(data.openThreshold));
      Serial.println(F("MOC -> eLatch Open"));
      // elatch.open();  //user had pulled the lever with intention to open
      setDoorHandleState(DOOR_HANDLE_OPEN);
      triggerActive = true;
    }
  } else
    // resting position of door handle
    if (triggerActive && (abs(data.diff) < data.unlockThreshold)) {
      // Reset when returning to rest state
      Serial.println(F("MOC -> released"));
      triggerActive = false;
      // elatch.lock();  //come back to lock state
    }
}

void updateeLatchSwitch(void) {
  bool rawState = digitalRead(E_LATCH_SW_PIN);
  risingSwEdge = (rawState && !currentSwState);
  fallingSwEdge = (!rawState && currentSwState);
  lastSwState = currentSwState;
  currentSwState = rawState;
  if (lastSwState != currentSwState) {
    Serial.println(String(F("Switch status changed from ")) + String(lastSwState) + String(F(" to ")) + String(currentSwState));
  }
  latchSwitchState = currentSwState;
}


// === Setup ===
void setup() {

  //debugLog.begin(DEBUGGING_ENABLED, DebugLogger::VERBOSE);

  Serial.begin(SERIAL_DEBUG_SPEED);  //debugging
  delay(500);

  while (!Serial) {
    Serial.println(F("."));
    delay(100);
  }
  delay(100);

  Serial.println(F(""));
  Serial.println(F(""));
  Serial.println(F("AUDI E-Latch Demo Mockup 2"));
  Serial.println(F("==================================================="));
  Serial.println(F("ITW Automotive - SmartComponents EU"));
  Serial.println(F("Evaluation of MOC using mockup of door handle design"));
  Serial.println(F("and using AUDI e-Latch as combo"));
  Serial.println(F(" --------------------------------------------------"));
  Serial.println(F("by Adrian David, Smart Components Platform"));
  Serial.println(F("version: 1.3, 2025.08.22"));
  Serial.println(F("==================================================="));
  Serial.println(F(""));
  Serial.println(F(""));


  Serial.println(F("System Started."));

  //moc sensor
  mocReader.begin();
  mocReader.setChangeCallback(onMOCPull);

  //door handle actuator
  actuator.begin(MOTOR_ENABLE_PIN, MOTOR_RPWM_PIN, MOTOR_LPWM_PIN,
                 DEPLOY_PWM, RETRACT_PWM,
                 DEPLOY_TIME_MS, RETRACT_TIME_MS);

  // Relay to drive elacth
  eLatchMotorDriver.begin(RELAY_SW_CW_PIN, RELAY_SW_CCW_PIN);
  Nb_Open_Attempt = 0;

  // Elatch switch configuration
  pinMode(E_LATCH_SW_PIN, INPUT_PULLUP);


  pinMode(DEPLOY_SW_PIN, INPUT_PULLUP);
  pinMode(RETRACT_SW_PIN, INPUT_PULLUP);

  switchStatusDeploy = digitalRead(DEPLOY_SW_PIN);
  switchStatusRetract = digitalRead(RETRACT_SW_PIN);

  if (switchStatusDeploy) Serial.println(F(" Direct read:: Deploy is pressed"));

  if (switchStatusRetract) Serial.println(F(" Direct read:: Retract is pressed"));

  //capa sensor
  extcapaSensor.begin(EXT_CAPA_PWR_PIN, EXT_CAPA_SEN_PIN);
  inrcapaSensor.begin(INR_CAPA_PWR_PIN, INR_CAPA_SEN_PIN);

  // MOC deploy / retract user threshold settings
  userPotiDeploy.begin(ADC_USER_DEPLOY_PIN, NUM_SAMPLES, ADC_REF_VOLTAGE);
  userPotiRetract.begin(ADC_USER_RETRACT_PIN, NUM_SAMPLES, ADC_REF_VOLTAGE);

  //led door handle
  ledCtrl.begin(LED_PWM_PIN, LED_MAX_BRIGHTNESS);

  // initialization of main state machine
  doorHandleState = DOOR_HANDLE_INIT;
  // lastDoorHandleState = DOOR_HANDLE_INIT;

  Serial.println(F("Setup Completed."));
  Serial.println(F(""));
}  // end setup

void RefreshHandleState(void);

// === Main Loop ===
void loop(void) {

  buttonDeploy.update();
  buttonRetract.update();

  // switchStatusDeploy = digitalRead(DEPLOY_SW_PIN);
  // switchStatusRetract = digitalRead(RETRACT_SW_PIN);

  // Serial.println(" Direct read:: Switch Status : " + String(switchStatusDeploy) + " " + String(switchStatusRetract));

  // if (buttonDeploy.getswitchStatus()) {
  //   Serial.println(" Debounce:: Deploy Switch is pressed : " + String(buttonDeploy.isPressed()) + "  " + String(buttonDeploy.isUp()) + "  " + String(buttonDeploy.isDown()) + "  " + String(buttonDeploy.isReleased()) + "  " + String(buttonDeploy.getswitchStatus()) +"  " + String(switchStatusDeploy) );
  //   Serial.println(" Debounce:: Retract Switch is pressed : " + String(buttonRetract.isPressed()) + "  " + String(buttonRetract.isUp()) + "  " + String(buttonRetract.isDown()) + "  " + String(buttonRetract.isReleased()) + "  " + String(buttonRetract.getswitchStatus()) +"  " + String(switchStatusRetract));
  // } else if (buttonRetract.getswitchStatus()) {
  //   Serial.println(" Debounce:: Deploy Switch is pressed : " + String(buttonDeploy.isPressed()) + "  " + String(buttonDeploy.isUp()) + "  " + String(buttonDeploy.isDown()) + "  " + String(buttonDeploy.isReleased()) + "  " + String(buttonDeploy.getswitchStatus()));
  //   Serial.println(" Debounce:: Retract Switch is pressed : " + String(buttonRetract.isPressed()) + "  " + String(buttonRetract.isUp()) + "  " + String(buttonRetract.isDown()) + "  " + String(buttonRetract.isReleased()) + "  " + String(buttonRetract.getswitchStatus()));
  // }


  // buttonDeploy = digitalRead(DEPLOY_SW_PIN);
  // buttonRetract = digitalRead(RETRACT_SW_PIN);

  // if(buttonDeploy) Serial.println(F(" Direct read:: Deploy is pressed"));

  // if (buttonRetract) Serial.println(F(" Direct read:: Retract is pressed"));

  updateeLatchSwitch();

  userPotiDeploy.update();
  userPotiRetract.update();

  mocReader.update();

  // Capa sensor update
  inrcapaSensor.update();
  extcapaSensor.update();

  ledCtrl.updateLedState();

  // MainStateMachineRun();
  RefreshHandleState();

  ledCtrl.updateLedState();
  // Serial.println("Current Handle state :: " + String(doorHandleState));

  // refresh the actuator state
  actuator.update();

  ledCtrl.updateLedState();

  // refresh elatch motor
  eLatchMotorDriver.update();



  //hbridge.setState(HBRIDGE_CW);    // Starts CW movement with these parameters
}  //end main loop

void Check_Disable_Locking(void) {
  // Handling the Proximity sensing logic to diable locking
  if ((!Disable_Locking) && inrcapaSensor.getCurrentState()) {
    extcapaSensor.enable(false);
    Disable_Locking = true;

  } else if (Disable_Locking && (!inrcapaSensor.getCurrentState())) {
    extcapaSensor.enable(true);
    Disable_Locking = false;
  }
  // Capa sensor update
  inrcapaSensor.update();
  extcapaSensor.update();
}

// Vaidation of DOOR Handle state transitions with previous state
void setDoorHandleState(DoorHandleState state) {

  if (state != doorHandleState) {
    switch (state) {
      case DOOR_HANDLE_INIT:
        break;
      case DOOR_HANDLE_CLOSED:
        if (latchSwitchState && ((doorHandleState == DOOR_HANDLE_INIT) || (doorHandleState == DOOR_HANDLE_RETRACT))) {
          doorHandleState = state;
          ledCtrl.ledOn();
          Serial.println(F("Entering DOOR_HANDLE_CLOSED"));
        } else {
          Serial.println(F("Please close the DOOR!!!"));
          ledCtrl.ledOff();
        }
        break;
      case DOOR_HANDLE_RETRACT:
        if ((doorHandleState == DOOR_HANDLE_LATCHED) || (doorHandleState == DOOR_HANDLE_WAIT_OPEN)) {
          doorHandleState = state;
          Serial.println(F("Entering DOOR_HANDLE_RETRACT"));
        } else {
          Serial.println(F("Cannot Retract DOOR HANDLE!!!"));
        }
        break;
      case DOOR_HANDLE_DEPLOYED:
        if (doorHandleState == DOOR_HANDLE_CLOSED) {
          doorHandleState = state;
          Serial.println(F("Entering DOOR_HANDLE_DEPLOYED"));
        } else {
          Serial.println("Cannot Deploy DOOR HANDLE!!! from " + String(doorHandleState));
        }
        break;
      case DOOR_HANDLE_WAIT_OPEN:
        if (doorHandleState == DOOR_HANDLE_DEPLOYED) {
          doorHandleState = state;
          Serial.println(F("Entering DOOR_HANDLE_WAIT_OPEN"));
        } else {
          Serial.println(F("Deployment of DOOR HANDLE not correct!!!"));
        }
        break;
      case DOOR_HANDLE_OPEN:
        if ((doorHandleState == DOOR_HANDLE_WAIT_OPEN) || (doorHandleState == DOOR_HANDLE_LATCHED)) {
          doorHandleState = state;
          Nb_Open_Attempt = 0;
          Serial.println(F("Entering DOOR_HANDLE_OPEN"));
        } else {
          Serial.println(F("Opening of DOOR HANDLE is not correct at this moment!!!"));
        }
        break;
      case DOOR_HANDLE_WAIT_TO_LATCH:
        if (doorHandleState == DOOR_HANDLE_OPEN) {
          doorHandleState = state;
          Serial.println(F("Entering DOOR_HANDLE_WAIT_TO_LATCH"));
        } else {
          Serial.println(F("Latching of DOOR HANDLE before Open is not correct!!!"));
        }
        break;
      case DOOR_HANDLE_LATCHED:
        if (doorHandleState == DOOR_HANDLE_WAIT_TO_LATCH) {
          doorHandleState = state;
          Serial.println(F("Entering DOOR_HANDLE_LATCHED"));
        }
        break;
      default: Serial.println(F("Unknown state!!!")); break;
    }
  }

}  // setDoorHandleState

// void MainStateMachineRun(void) {
//   // Validate the state transitions
//   if (doorHandleState != lastDoorHandleState) {
//     switch (doorHandleState) {
//       case DOOR_HANDLE_INIT:
//         break;
//       case DOOR_HANDLE_CLOSED:
//         if (latchSwitchState) {
//           lastDoorHandleState = doorHandleState;
//           ledCtrl.ledOn();
//           Serial.println(F("Entering DOOR_HANDLE_CLOSED"));
//         } else {
//           Serial.println(F("Please close the DOOR!!!"));
//           ledCtrl.ledOff();
//         }
//         break;
//       case DOOR_HANDLE_RETRACT:
//         if ((lastDoorHandleState != DOOR_HANDLE_LATCHED) || (lastDoorHandleState != DOOR_HANDLE_WAIT_OPEN)) {
//           Serial.println(F("Cannot Retract DOOR HANDLE!!!"));
//         } else {
//           lastDoorHandleState = doorHandleState;
//           Serial.println(F("Entering DOOR_HANDLE_RETRACT"));
//         }
//         break;
//       case DOOR_HANDLE_DEPLOYED:
//         if (lastDoorHandleState == DOOR_HANDLE_CLOSED) {
//           lastDoorHandleState = doorHandleState;
//           Serial.println(F("Entering DOOR_HANDLE_DEPLOYED"));
//         } else {
//           Serial.println(F("Cannot Deploy DOOR HANDLE!!!"));
//         }
//         break;
//       case DOOR_HANDLE_WAIT_OPEN:
//         if (lastDoorHandleState == DOOR_HANDLE_DEPLOYED) {
//           lastDoorHandleState = doorHandleState;
//           Serial.println(F("Entering DOOR_HANDLE_WAIT_OPEN"));
//         } else {
//           Serial.println(F("Deployment of DOOR HANDLE not correct!!!"));
//         }
//         break;
//       case DOOR_HANDLE_OPEN:
//         if (lastDoorHandleState == DOOR_HANDLE_WAIT_OPEN) {
//           lastDoorHandleState = doorHandleState;
//           Serial.println(F("Entering DOOR_HANDLE_OPEN"));
//         } else {
//           Serial.println(F("Opening of DOOR HANDLE before deployment is not correct!!!"));
//         }
//         break;
//       case DOOR_HANDLE_WAIT_TO_LATCH:
//         if (lastDoorHandleState == DOOR_HANDLE_OPEN) {
//           lastDoorHandleState = doorHandleState;
//           Serial.println(F("Entering DOOR_HANDLE_WAIT_TO_LATCH"));
//         } else {
//           Serial.println(F("Latching of DOOR HANDLE before Open is not correct!!!"));
//         }
//         break;
//       case DOOR_HANDLE_LATCHED:
//         if (lastDoorHandleState == DOOR_HANDLE_WAIT_TO_LATCH) {
//           lastDoorHandleState = doorHandleState;
//           Serial.println(F("Entering DOOR_HANDLE_LATCHED"));
//         }
//         break;
//       default: Serial.println(F("Unknown state entered")); break;
//     }
//   }

//   RefreshHandleState();
// }

void RefreshHandleState(void) {
  // refresh the state machine
  switch (doorHandleState) {
    case DOOR_HANDLE_INIT:
      if (latchSwitchState && actuator.setState(MOTOR_START_RETRACT) && eLatchMotorDriver.setState(RELAY_CCW))
        setDoorHandleState(DOOR_HANDLE_CLOSED);
      else
        Serial.println(F("DOOR_HANDLE_INIT:: Waiting for DOOR close status"));
      inrcapaSensor.enable(false);
      inrcapaSensor.enable(true);
      extcapaSensor.enable(false);
      extcapaSensor.enable(true);
      break;

    case DOOR_HANDLE_CLOSED:
      // Process Deploy switch event, check for MOTOR status then move to next state.
      // if (buttonDeploy.isPressed() && (actuator.getState() == MOTOR_STOP)) {
      if (buttonDeploy.getswitchStatus() && (actuator.getState() == MOTOR_STOP)) {
        setDoorHandleState(DOOR_HANDLE_DEPLOYED);
      } else {
        buttonDeploy.update();
        // Serial.println("DOOR_HANDLE_CLOSED:: Waiting for Trigger :: " + String(buttonDeploy.getswitchStatus()) + " " + String(actuator.getState()));
      }
      if (ledCtrl.getState() == LED_OFF)
        ledCtrl.ledOn();
      break;

    case DOOR_HANDLE_RETRACT:

      if (actuator.setState(MOTOR_START_RETRACT) && eLatchMotorDriver.setState(RELAY_CCW)) {
        Serial.println(F("DOOR_HANDLE_RETRACT:: CCW Triggered"));
        inrcapaSensor.enable(false);
        extcapaSensor.enable(false);
        setDoorHandleState(DOOR_HANDLE_CLOSED);
      } else {
        Serial.println(F("DOOR_HANDLE_RETRACT:: Waiting for Actuator & elatch status!!!"));
      }
      break;

    case DOOR_HANDLE_DEPLOYED:

      // actuator.triggerAction(4000);
      // go to next state only if the actuator and eLatch are deployed and ready
      if (actuator.setState(MOTOR_START_DEPLOY) && eLatchMotorDriver.setState(RELAY_CW)) {
        Serial.println(F("DOOR_HANDLE_DEPLOYED:: CW Triggered"));
        inrcapaSensor.enable(false);
        inrcapaSensor.enable(true);
        extcapaSensor.enable(false);
        extcapaSensor.enable(true);
        setDoorHandleState(DOOR_HANDLE_WAIT_OPEN);
      } else {
        Check_Disable_Locking();
        Serial.println(F("DOOR_HANDLE_DEPLOYED:: Waiting for Actuator & elatch status!!!"));
      }
      break;

    case DOOR_HANDLE_WAIT_OPEN:
      Check_Disable_Locking();

      // Incase of Retract switch pressed or external lock capa sensor pressed, retract the handle
      if (latchSwitchState && (buttonRetract.getswitchStatus() || extcapaSensor.getCurrentState())) {
        setDoorHandleState(DOOR_HANDLE_RETRACT);
      } else {  // just fade in and fade out LED
        if (ledCtrl.getState() == LED_ON)
          ledCtrl.fadeLedIn(LED_FADE_IN_TIME_MS);
        else if (ledCtrl.getState() == LED_OFF)
          ledCtrl.fadeLedOut(LED_FADE_OUT_TIME_MS);
      }
      break;

    case DOOR_HANDLE_OPEN:
      if (latchSwitchState) {
        if (Nb_Open_Attempt <= NB_OPEN_RETRY_COUNT) {
          if ((eLatchMotorDriver.getState() == RELAY_STOP) && eLatchMotorDriver.setState(RELAY_CW)) {
            if (ledCtrl.getState() == LED_OFF)
              ledCtrl.ledOn();
            ++Nb_Open_Attempt;
            Serial.println(F("DOOR_HANDLE_OPEN:: Waiting for eLatch to open!!!"));
          }
        }
      } else if ((eLatchMotorDriver.getRecentRun() == RELAY_CW) && (eLatchMotorDriver.getState() == RELAY_STOP)) {
        if (ledCtrl.getState() == LED_ON)
          ledCtrl.ledOff();
        Check_Disable_Locking();
        setDoorHandleState(DOOR_HANDLE_WAIT_TO_LATCH);
        Serial.println(F("DOOR_HANDLE_OPEN:: Waiting for DOOR latched!!!"));
      }
      break;
    case DOOR_HANDLE_WAIT_TO_LATCH:
      inrcapaSensor.enable(false);
      extcapaSensor.enable(false);

      if (latchSwitchState) {
        if (ledCtrl.getState() == LED_OFF)
          ledCtrl.ledOn();
        setDoorHandleState(DOOR_HANDLE_LATCHED);
      } else {
        if (ledCtrl.getState() == LED_ON)
          ledCtrl.ledOff();
      }
      break;
    case DOOR_HANDLE_LATCHED:
      inrcapaSensor.enable(true);
      extcapaSensor.enable(true);
      Check_Disable_Locking();

      // Incase of Retract switch pressed or external lock capa sensor pressed, retract the handle
      if (latchSwitchState && (buttonRetract.getswitchStatus() || extcapaSensor.getCurrentState())) {
        setDoorHandleState(DOOR_HANDLE_RETRACT);
      } else {  // just fade in and fade out LED
        if (ledCtrl.getState() == LED_ON)
          ledCtrl.fadeLedIn(LED_FADE_IN_TIME_MS);
        else if (ledCtrl.getState() == LED_OFF)
          ledCtrl.fadeLedOut(LED_FADE_OUT_TIME_MS);
      }
      break;

    default:
      Serial.println(F("Main state machine error"));
      break;
  }
}