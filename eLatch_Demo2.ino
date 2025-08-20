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
 *                     [D2] ------> CAPA Sensor PWR VDD                 
 *                     [D3] <------ CAPA Sensor OUT                            
 *                     [D4] ------> ELatch         
 *                     [D5]     x
 *                     [D6]     x                            
 *                     [D7]     x
 *                     [D8] ------> H-Bridge Enable
 *                     [D9] ------> H-Bridge RPWM
 *                     [D10]------> H-Bridge LPWM
 *                     [D11]------> 
 *                     [D12]------> ELatch Switch
 *                     [A0] ------> Not Used
 *                     [A1] <------ Unlock Threshold                         
 *                     [A2] <------ Open Threshold                        
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
//#include "DebugLogger.h"
//#include "ADCReader.h"
//#include "global.h"
//#include "HBridge.h"
#include "ELatchController.h"
#include "CapaTouchSensor.h"

#define DEBUGGING_ENABLED true
#define SERIAL_DEBUG_SPEED 19200


// ==== Manual Control of Door Handle Pin Switch Settings ====
#define SW_CW_PIN 6
#define SW_CCW_PIN 7

//=== Manual Control of Door Handle Pin Switch ===
Debounce buttonCW(SW_CW_PIN);
Debounce buttonCCW(SW_CCW_PIN);


void onMOCPull(const MOCSignalData& data) {

  static bool triggerActive = false;
  static bool unlockTriggerActive = false;

  // Serial.println(String(F("data.diff: ")) + String(abs(data.diff)));
  // Serial.println(String(F("unlockThreshold: ")) + String(data.unlockThreshold));
  // Serial.println(String(F("openThreshold: ")) + String(data.openThreshold));


  //detect user action like pulling
  if ((!unlockTriggerActive) && abs(data.diff) >= data.unlockThreshold) {

    // bool monotonicOpen = ((data.mid <= data.oldest) && (data.newest <= data.mid));
    // if diff exceeds unlock threshold then call unlock
    Serial.println(String(F("data.diff: ")) + String(abs(data.diff)));
    Serial.println(String(F("unlockThreshold: ")) + String(data.unlockThreshold));
    // Serial.println(String(F("openThreshold: ")) + String(data.openThreshold));
    Serial.println(F("MOC -> eLatch unlocked!"));
    elatch.unlock();
    unlockTriggerActive = true;

  } else if (!triggerActive && abs(data.diff) >= data.unlockThreshold) {

    bool monotonicOpen = ((data.mid <= data.oldest) && (data.newest <= data.mid));
    bool openTrig = ((data.diff <= -data.openThreshold) && monotonicOpen);
    if (openTrig) {
      Serial.println(String(F("data.diff: ")) + String(abs(data.diff)));
      // Serial.println(String(F("unlockThreshold: ")) + String(data.unlockThreshold));
      Serial.println(String(F("openThreshold: ")) + String(data.openThreshold));
      Serial.println(F("MOC -> eLatch Open"));
      // elatch.open();  //user had pulled the lever with intention to open
      triggerActive = true;
    }
  } else
    // resting position of door handle
    if (unlockTriggerActive && abs(data.diff) < data.unlockThreshold) {
      // Reset when returning to rest state
      Serial.println(F("MOC -> eLatch locked"));
      triggerActive = false;
      unlockTriggerActive = false;
      elatch.lock();  //come back to lock state
    }
}

// bool toogle = true;
// bool toogleLast = false;
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

  Serial.println(F(""));
  Serial.println(F(""));
  Serial.println(F("AUDI E-Latch Demo Mockup 2"));
  Serial.println(F("==================================================="));
  Serial.println(F("ITW Automotive - SmartComponents EU"));
  Serial.println(F("Evaluation of MOC using mockup of door handle design"));
  Serial.println(F("and using AUDI e-Latch as combo"));
  Serial.println(F(" --------------------------------------------------"));
  Serial.println(F("by Adrian David, Smart Components Platform"));
  Serial.println(F("version: 1.2, 2025.07.28"));
  Serial.println(F("==================================================="));
  Serial.println(F(""));
  Serial.println(F(""));


  Serial.println(F("System Started."));


  //moc sensor
  mocReader.begin();
  mocReader.setChangeCallback(onMOCPull);

  Serial.println(String(F("Hbridge state before change: ")) + String(hbridge.getState()));

  //h-bridge
  hbridge.begin(HBRIDGE_ENABLE_PIN, HBRIDGE_RPWM_PIN, HBRIDGE_LPWM_PIN);

  //e-Latch
  elatch.begin(&hbridge, E_LATCH_SW_PIN);

  //capa sensor
  capaSensor.begin(CAPA_PWR_PIN, CAPA_SEN_PIN);

  // MOC deploy / retract user threshold settings
  userPotiDeploy.begin(ADC_USER_DEPLOY_PIN, NUM_SAMPLES, ADC_REF_VOLTAGE);
  userPotiRetract.begin(ADC_USER_RETRACT_PIN, NUM_SAMPLES, ADC_REF_VOLTAGE);


  Serial.println(F("Setup Completed."));
  Serial.println(F(""));
  Serial.println(String(F("Hbridge state before change: ")) + String(hbridge.getState()));

  elatch.enableActivation(true);
  // Serial.println(F("Hbridge state after change: ") + String(hbridge.getState()));
  Serial.println(String(F("Hbridge state after change: ")) + String(hbridge.getState()));



}  // end setup





// === Main Loop ===
void loop(void) {

  buttonCW.update();
  buttonCCW.update();
  // eLatchSwitch.update();

  elatch.update();  //hbridge update is called inside of elatch update

  userPotiDeploy.update();
  userPotiRetract.update();

  mocReader.update();

  capaSensor.update();

  if(capaSensor.getCurrentState())
  {
    Serial.println("CAPA Sensor state:: Pressed");
  }

  //hbridge.setState(HBRIDGE_CW);    // Starts CW movement with these parameters
}  //end main loop
