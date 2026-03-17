#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <esp32_port.h>

// MCU PIN Usage/ Layout


#define MOTOR2_ENABLE_PIN 22  // OUT: eLatch enable
#define MOTOR2_RPWM_PIN   21  // OUT: eLatch CW
#define MOTOR2_LPWM_PIN   19  // OUT: eLatch CCW
#define E_LATCH_SW_PIN    23  // IN: eLatch Switch read

#define MOTOR1_ENABLE_PIN 25  // OUT: Actuator enable
#define MOTOR1_RPWM_PIN   26  // OUT: Actuator CW
#define MOTOR1_LPWM_PIN   27  // OUT: Actuator CCW

#define DEPLOY_SW_PIN     33  // IN: Deploy Switch read
#define RETRACT_SW_PIN    32  // IN: Retract Switch read
#define OPEN_SWITCH_PIN   35  // IN: Open Switch read (input-only)
// #define DEPLOY_HANDLE_SW_PIN 39 // IN: Deploy Switch (teramount) read (VN)

#define ADC_USER_OPEN_THRESHOLD_PIN 34 // IN: Threshold reading for deployment

#define ILLUMINATION_LED_PIN 18 // OUT: LED Illumination
#define LED_PWM_PIN 2          // OUT: LED indications(Strip)
#define NUM_PIXELS 10     // Number of LEDs in the strip
/*Status LEDs
0. Power - RED
1. Flush(Deploy) - blink during flush
2. LOCK PROX - 
3. UNLOCK PROX -
4. OPEN Signal - GREEN
5. Door Lock Status - RED
*/

#define SERIAL1_RX_PIN 16
#define SERIAL1_TX_PIN 17
#define SERIAL1_UART_NUM UART_NUM_1
#define SERIAL1_RX_BUF_SIZE 1024
#define SERIAL1_EVENT_QUEUE_SZ 20

// ESP32 Pinning(GPIO):
// 2 LED indication via Strip
// 34 Poti1
// D2 Ideneo board read(rx)
// D3 Ideneo board write(tx)
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


#define MAX_ATTEMPTS 2  // Max attempts allowed to protect motor life cycle


// MOC Reading constants
#define MOC_COM_SPEED 19200
#define CONNECTION_MOC_TIMEOUT_MS 500
#define TIMEOUT_MOC_CONNECTION 1

#define PACKET_BUFFER_SIZE 64
#define SAMPLE_COUNT 1

#define MOC_UNLOCK_THRESHOLD_MIN 300
#define MOC_UNLOCK_THRESHOLD_MAX 2100
#define MOC_OPEN_THRESHOLD_MIN 2101
#define MOC_OPEN_THRESHOLD_MAX 5000


//Timeout Manager constants
#define MAX_TIMEOUTS 5

#define ELATCH_MOTOR_RUN_TIME_CW 200
#define ELATCH_MOTOR_RUN_TIME_CCW 200
#define ELATCH_MOTOR_RUN_PWM 3000

#define ELATCH_MOTOR_STOP_TIME 500

#define ADC_REF_VOLTAGE 3.3
#define NUM_SAMPLES 16

#define LED_MAX_BRIGHTNESS 25
#define LED_FADE_IN_TIME_MS 1500
#define LED_FADE_OUT_TIME_MS 600

#define RETRACT_TIME_MS 1400
#define RETRACT_PWM 3600

#define DEPLOY_TIME_MS 1000
#define DEPLOY_PWM 3600

#define NB_OPEN_RETRY_COUNT 2

// --- UART driver/task globals (were referenced but not defined) ---
#ifndef SERIAL1_UART_NUM
#define SERIAL1_UART_NUM UART_NUM_1
#endif
#ifndef SERIAL1_RX_PIN
#define SERIAL1_RX_PIN 16
#endif
#ifndef SERIAL1_TX_PIN
#define SERIAL1_TX_PIN 17
#endif
#ifndef SERIAL1_RX_BUF_SIZE
#define SERIAL1_RX_BUF_SIZE 1024
#endif
#ifndef SERIAL1_TX_BUF_SIZE
#define SERIAL1_TX_BUF_SIZE 512
#endif
#ifndef SERIAL1_EVENT_QUEUE_SZ
#define SERIAL1_EVENT_QUEUE_SZ 20
#endif

#define RING_SIZE 512

#endif