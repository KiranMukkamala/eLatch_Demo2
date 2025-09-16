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
 *                     [A0] ------> LED 
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
 * 2025.08.19 - v1.3 Integration with CAPA Sensors, Actuators and Feedback Mechanisms
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
#include "DoorHandleController.h"

#define DEBUGGING_ENABLED false
#define SERIAL_DEBUG_SPEED 19200

/*====  Main Door Handle Controller object ====*/
DoorHandleController doorHandleController;

/*==== Object for Actuator motor driver ===*/
MotorController actuator;

/*==== Object for elatch motor driver ===*/
RelayController eLatchMotorDriver;

//=== Deploy and Retract Switch Objects ===
Debounce buttonDeploy(DEPLOY_SW_PIN, LOW);
Debounce buttonRetract(RETRACT_SW_PIN, LOW);

Debounce buttonDoorHandleDeploy(DEPLOY_HANDLE_SW_PIN, LOW);

//CAPA Sensor objects
CapaTouchSensor extcapaSensor;
CapaTouchSensor inrcapaSensor;

//LED control object
LEDControl ledCtrl;
LEDControl ledLockStatus;
LEDControl ledCapaStatus;

void onMOCPull(const MOCSignalData& data) {

  static bool triggerActive = false;
  bool monotonicOpen = ((data.mid <= data.oldest) && (data.newest <= data.mid));
  //static bool unlockTriggerActive = false;

  // Serial.println(String(F("data.diff: ")) + String(abs(data.diff)));
  // Serial.println(String(F("unlockThreshold: ")) + String(data.unlockThreshold));
  // Serial.println(String(F("openThreshold: ")) + String(data.openThreshold));


  //detect user action like pulling
  if (!triggerActive && abs(data.diff) >= data.unlockThreshold) {

    if (monotonicOpen) {
      // Serial.println(String(F("data.diff: ")) + String(abs(data.diff)));
      // Serial.println(String(F("unlockThreshold: ")) + String(data.unlockThreshold));
      // Serial.println(String(F("openThreshold: ")) + String(data.openThreshold));
      // Serial.println(F("MOC -> eLatch Open"));
      // elatch.open();  //user had pulled the lever with intention to open
      doorHandleController.setState(DOOR_HANDLE_OPEN);
      triggerActive = true;
    }
  } else
    // resting position of door handle
    if (triggerActive && (abs(data.diff) < data.unlockThreshold)) {
      // Reset when returning to rest state
      // Serial.println(F("MOC -> released"));
      triggerActive = false;
      // elatch.lock();  //come back to lock state
    }

  // Serial Port Plotter v1.3.0 - Data Output
  // Door Lock Status
  Serial.print("DOOR_LOCK_STATUS:");
  Serial.print(doorHandleController.getswitchStatus() ? data.unlockThreshold + 500 : data.unlockThreshold + 450);

  // External Capacitive Sensor Status
  Serial.print("\tEXT_CAPA_STATUS:");
  Serial.print(extcapaSensor.getCurrentState() ? data.unlockThreshold + 350 : data.unlockThreshold + 300);

  // Internal Capacitive Sensor Status
  Serial.print("\tINN_CAPA_STATUS:");
  Serial.print(inrcapaSensor.getCurrentState() ? data.unlockThreshold + 200 : data.unlockThreshold + 100);

  // Deploy Switch Status
  Serial.print("\tDEPLOY_STATUS:");
  Serial.print(buttonDoorHandleDeploy.getswitchStatus() ? data.unlockThreshold + 600 : data.unlockThreshold + 550);

  // Threshold and Pressure Data
  Serial.print("\tMOC_THRESHOLD:");
  Serial.print(data.unlockThreshold);

  Serial.print("\tMOC_PRESSURE:");
  Serial.println(abs(data.diff));
}

// === Setup ===
void setup() {

  //debugLog.begin(DEBUGGING_ENABLED, DebugLogger::VERBOSE);

  Serial.begin(SERIAL_DEBUG_SPEED);  //debugging
  delay(500);

  // while (!Serial) {
  //   Serial.println(F("."));
  //   delay(100);
  // }
  // delay(100);

  // Serial.println(F(""));
  // Serial.println(F(""));
  // Serial.println(F("AUDI E-Latch Demo Mockup 2"));
  // Serial.println(F("==================================================="));
  // Serial.println(F("ITW Automotive - SmartComponents EU"));
  // Serial.println(F("Evaluation of MOC using mockup of door handle design"));
  // Serial.println(F("and using AUDI e-Latch as combo"));
  // Serial.println(F(" --------------------------------------------------"));
  // Serial.println(F("by Adrian David, Smart Components Platform"));
  // Serial.println(F("version: 1.3, 2025.08.22"));
  // Serial.println(F("==================================================="));
  // Serial.println(F(""));
  // Serial.println(F(""));


  // Serial.println(F("System Started."));

  // MOC deploy / retract user threshold settings
  userPotiDeploy.begin(ADC_USER_DEPLOY_PIN, NUM_SAMPLES, ADC_REF_VOLTAGE);
  userPotiRetract.begin(ADC_USER_RETRACT_PIN, NUM_SAMPLES, ADC_REF_VOLTAGE);

  //moc sensor
  mocReader.begin();
  mocReader.setChangeCallback(onMOCPull);

  //door handle actuator
  actuator.begin(MOTOR_ENABLE_PIN, MOTOR_RPWM_PIN, MOTOR_LPWM_PIN,
                 DEPLOY_PWM, RETRACT_PWM,
                 DEPLOY_TIME_MS, RETRACT_TIME_MS);

  // Relay to drive elacth
  eLatchMotorDriver.begin(RELAY_SW_CW_PIN, RELAY_SW_CCW_PIN);

  // Elatch switch configuration
  pinMode(E_LATCH_SW_PIN, INPUT_PULLUP);

  // Configure the PINS for usage
  pinMode(DEPLOY_SW_PIN, INPUT_PULLUP);
  pinMode(RETRACT_SW_PIN, INPUT_PULLUP);
  pinMode(DEPLOY_HANDLE_SW_PIN, INPUT_PULLUP);

  //capa sensor
  extcapaSensor.begin(EXT_CAPA_PWR_PIN, EXT_CAPA_SEN_PIN);
  inrcapaSensor.begin(INR_CAPA_PWR_PIN, INR_CAPA_SEN_PIN);

  //led door handle
  ledCtrl.begin(LED_PWM_PIN, LED_MAX_BRIGHTNESS);
  ledLockStatus.begin(LED_LOCK_STATUS_PIN, LED_MAX_BRIGHTNESS);
  ledCapaStatus.begin(LED_CAPA_STATUS_PIN, LED_MAX_BRIGHTNESS);


  // Door Handle Controller object configuration
  doorHandleController.setDependencies(&buttonDeploy, &buttonRetract, &buttonDoorHandleDeploy, &extcapaSensor, &inrcapaSensor, &ledCtrl, &actuator, &eLatchMotorDriver);

  // Serial.println(F("Setup Completed."));
  // Serial.println(F(""));
}  // end setup

// void RefreshHandleState(void);

// === Main Loop ===
void loop(void) {
  unsigned long t_start = micros();
  unsigned long t_button = 0, t_latch = 0, t_capa = 0, t_poti = 0, t_moc = 0;
  unsigned long t_state = 0, t_motor = 0, t_actuator = 0, t_led = 0;
  unsigned long t_end = 0;

  unsigned long t0 = micros();
  buttonDeploy.update();
  buttonRetract.update();
  buttonDoorHandleDeploy.update();

  t_button = micros() - t0;

  t0 = micros();
  doorHandleController.updateeLatchSwitch();
  t_latch = micros() - t0;

  t0 = micros();
  inrcapaSensor.update();
  extcapaSensor.update();
  t_capa = micros() - t0;

  t0 = micros();
  userPotiDeploy.update();
  // userPotiRetract.update();
  t_poti = micros() - t0;

  t0 = micros();
  mocReader.update();
  t_moc = micros() - t0;

  t0 = micros();
  doorHandleController.refreshState();
  t_state = micros() - t0;

  t0 = micros();
  eLatchMotorDriver.update();
  t_motor = micros() - t0;

  t0 = micros();
  actuator.update();
  t_actuator = micros() - t0;

  t0 = micros();
  ledCtrl.updateLedState();
  if (doorHandleController.getswitchStatus())
    ledLockStatus.ledOff();
  else
    ledLockStatus.ledOn();

  ledLockStatus.updateLedState();

  if (inrcapaSensor.getCurrentState())
    ledCapaStatus.ledOn();
  else
    ledCapaStatus.ledOff();
  ledCapaStatus.updateLedState();
  t_led = micros() - t0;

  t_end = micros();

#if DEBUGGING_ENABLED
  // if ((t_end - t_start) > 500) {
  // Serial.print("Timing(us): ");
  Serial.print("Buttons:");
  Serial.print(t_button);
  Serial.print("\t LatchSw:");
  Serial.print(t_latch);
  Serial.print("\t Capa:");
  Serial.print(t_capa);
  Serial.print("\t Potis:");
  Serial.print(t_poti);
  Serial.print("\t MOC:");
  Serial.print(t_moc);
  Serial.print("\t HandleState:");
  Serial.print(t_state);
  Serial.print("\t Motor:");
  Serial.print(t_motor);
  Serial.print("\t Actuator:");
  Serial.print(t_actuator);
  Serial.print("\t LED:");
  Serial.print(t_led);
  Serial.print("\t Total:");
  Serial.println(t_end - t_start);
  // }
#endif
}  //end main loop