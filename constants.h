#ifndef CONSTANTS_H
#define CONSTANTS_H

// MCU PIN Usage/ Layout

#define EXT_CAPA_PWR_PIN 11  // OUT: External CAPA Sensor enable
#define EXT_CAPA_SEN_PIN 5  // IN: External CAPA Sensor read

#define MOTOR_ENABLE_PIN 8  // OUT: Actuator enable
#define MOTOR_RPWM_PIN 9    // OUT: Actuator CW
#define MOTOR_LPWM_PIN 10   // OUT: Actuator CCW
#define E_LATCH_SW_PIN 12   // IN: eLatch Switch read

#define INR_CAPA_PWR_PIN 2  // OUT: Inner CAPA Sensor enable
#define INR_CAPA_SEN_PIN 3   // IN: Inner CAPA Sensor read

#define DEPLOY_SW_PIN 6   //IN: Deploy Switch read
#define RETRACT_SW_PIN 7  //IN: Retract Switch read

#define LED_PWM_PIN A0  // OUT LED indications

#define RELAY_SW_CW_PIN 4    // CW
#define RELAY_SW_CCW_PIN A5  // CCW

// Arduino Pinning (cable color):
// A1 Poti1 blue
// A2 Poti2 white
// D2 Capa1 Read (red)
// D3 Capa1 Enable (white)
// D4 Actuator Deploy (int yellow)
// D5 Capa2 Read (dark blue, int white)
// D6 Switch Deploy (violett)
// D7 Switch Retract (blue)
// D8 eLatch Motor Enable (int)
// D9 eLatch RPWM (int)
// D10 eLatch LPWM (int)
// D11 Capa2 Enable (black, int yellow)
// D12 eLatch Switch Input (int)
// D23 Actuator Retract (int blue)
// A0 LED indication

// #define ELATCH_LOCK_TIMEOUT_MS 5000  // ms relaxation for lock detection
//#define ELATCH_UNLOCK_TIMEOUT_MS  800     // ms the latch motor runs to unlock
//#define ELATCH_OPEN_TIMEOUT_MS    1000    // ms for open (if needed)
//#define ELATCH_RETRIGGER_BLOCK_MS 1000    // ms block after operation

#define MAX_ATTEMPTS 2  // Max attempts allowed to protect motor life cycle


// MOC Reading constants

#define MOC_COM_SPEED 19200
#define CONNECTION_MOC_TIMEOUT_MS 500
#define TIMEOUT_MOC_CONNECTION 1

#define PACKET_BUFFER_SIZE 64
#define SAMPLE_COUNT 1

#define MOC_UNLOCK_THRESHOLD_MIN 300
#define MOC_UNLOCK_THRESHOLD_MAX 1800
#define MOC_OPEN_THRESHOLD_MIN 2101
#define MOC_OPEN_THRESHOLD_MAX 5000


//Timeout Manager constants
#define MAX_TIMEOUTS 5

// Timeout IDs
// #define TIMEOUT_DEPLOYED_DOOR_HANDLE 0
// //#define TIMEOUT_MOC_CONNECTION 1
// #define TIMEOUT_SERIAL_CONNECTION 2
// #define TIMEOUT_HANDLE_DEPLOY_DELAY 3
// #define TIMEOUT_E_LATCH 5



// Default timeout durations (optional)
// #define TIMEOUT_HANDLE_DEPLOY_DELAY_MS 200
// #define TIMEOUT_DEPLOYED_DOOR_HANDLE_MS 600000
// #define TIMEOUT_CONNECTION_SERIAL_TIMEOUT_MS 100
// #define TIMEOUT_E_LATCH_COOLDOWN_TIME_MS 1000


#define ELATCH_MOTOR_RUN_TIME_CW 300
#define ELATCH_MOTOR_RUN_TIME_CCW 200

#define ELATCH_MOTOR_STOP_TIME 500

// ADC Reader constants for Threshold reading
#define ADC_USER_DEPLOY_PIN A2
#define ADC_USER_RETRACT_PIN A1
#define ADC_REF_VOLTAGE 5
#define NUM_SAMPLES 10

#define LED_MAX_BRIGHTNESS 255
#define LED_FADE_IN_TIME_MS 1500
#define LED_FADE_OUT_TIME_MS 600

#define RETRACT_TIME_MS 800
#define RETRACT_PWM 900

#define DEPLOY_TIME_MS 800
#define DEPLOY_PWM 900

#define NB_OPEN_RETRY_COUNT 2

//=== DOOR Handle state ===
enum DoorHandleState {
  DOOR_HANDLE_INIT = 0,
  DOOR_HANDLE_CLOSED = 1,
  DOOR_HANDLE_DEPLOYED = 2,
  DOOR_HANDLE_WAIT_OPEN = 3,
  DOOR_HANDLE_OPEN = 4,
  DOOR_HANDLE_WAIT_TO_LATCH = 5,
  DOOR_HANDLE_LATCHED = 6,
  DOOR_HANDLE_RETRACT = 7
};

#endif