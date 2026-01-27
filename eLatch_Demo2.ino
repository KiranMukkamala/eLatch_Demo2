/**
 * @file eLatch_Demo2.ino
 * @brief Main application file for DH demonstrator.
 * 
 * This sketch reads the capacitive force using MOC technology from Microchip, 
 * detects threshold changes and activate E-Latch when specified threshold is reached.
 * 
 * @details
 * The system uses Serial1 to communicate with the MOC sensor module from Idneo. 
 * 
 * When a light threshold is detected elatch is unlocked, if the threshold is lower then this threshold elatch goes back to lock mode.
 * From Unlock mode if user increase the pressure on MOC sensor by pulling harder and open thresold is reached the elatch open command is issued.
 * 
 * 
 * 
 * @author
 * Adrian David/ Kiran Mukkamala
 * 
 * @date
 * 2026-01-27
 * 
 * @version
 * 2.3
 *
 * @note
 * Firmware is writen for ESP32 WROOM board. Debugging is via native Serial port(TX0-RX0).
 *
 * @bug
 * Starting up is not always corectly completed when debugging via serial is done. For demonstrator debugging must be deactivated.
 * ============================================================================
 *                               PINOUT DIAGRAM
 * ============================================================================
 *  
 * Refer the file located at: https://itwconnect.sharepoint.com/:u:/s/BODYINNOVATION-EU/IQDc576UdgzDR4En-ZPYNAK-Ad4s3TMtMm_02cHczzPD4YA?e=zyf2cb
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
 * 2025.10.02 - v2.0 A-Sample Integration with Ideo ECU for MOC, 2 CAPs reading
 * 2025.12.17 - v2.1 Updates for ESP32 WROOM upgrade
 * 2026.01.19 - v2.2 Updates requested from Roland, Deploy happens only with switch no more pulling.
 * 2026.01.27 - v2.3 Updates inlcude removal of actuator, teramont switch Since the handle is fixed in deploy pos.
 * ============================================================================
 * 
 */

#include "esp32-hal.h"
#include "esp_log.h"
#include "./Debounce.h"
#include "ADCReader.h"
#include "TimeoutManager.h"
#include "LEDControl.h"
#include "constants.h"
#include "motorController.h"
#include "DoorHandleController.h"
#include <driver/uart.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>

#define DEBUGGING_ENABLED false
#define SERIAL_DEBUG_SPEED 115200

/*====  Main Door Handle Controller object ====*/
DoorHandleController doorHandleController;

// NOTE: ensure only one definition of 'actuator' exists in the project.
// motorController.cpp should contain the concrete definition 'MotorController actuator;'
// Here we declare extern to avoid duplicate definition at link time.
// MotorController actuator;
// If eLatchMotorDriver is defined in another translation unit, declare extern.
// If not, create the single definition in one .cpp file instead of in the .ino.
MotorController eLatchMotorDriver;

/*=== Deploy and Retract Switch Objects ===*/
Debounce buttonDeploy(DEPLOY_SW_PIN, LOW);
Debounce buttonRetract(RETRACT_SW_PIN, LOW);
Debounce buttonOpenRemoteSwitch(OPEN_SWITCH_PIN, LOW);
// Debounce buttonDoorHandleDeploy(DEPLOY_HANDLE_SW_PIN, LOW);
// bool buttonDoorHandleDeploy = false;

// LED control objects
LEDControl ledCtrl;

// Field count for frames (keep small if possible)
constexpr uint8_t FIELD_COUNT = 12;

// MOC values Array
uint16_t values[FIELD_COUNT];

// UART frame
constexpr uint8_t maxFrameLength = 80;
char rawFrame[maxFrameLength];

// runtime flags
bool verbosePrint = false;  // set to true to print full frame and fields (slower)

// Timing control
bool frameTimingEnabled = false;  // print per-frame timing
unsigned long frameStartMicros = 0;
unsigned long frameEndMicros = 0;
unsigned long lastFrameEndMicros = 0;

// ISR ring buffer for Serial1 RX to improve robustness at high throughput
volatile uint8_t ring_buf[RING_SIZE];
volatile uint16_t ring_head = 0;  // next write index
volatile uint16_t ring_tail = 0;  // next read index (consumed by main)

// Frame position bookkeeping (positions are indices into ring_buf)
volatile uint8_t hook_receiving = 0;
volatile uint16_t hook_frame_start_pos = 0;
volatile uint16_t hook_frame_end_pos = 0;
volatile uint8_t hook_has_frame = 0;
volatile unsigned long hook_frame_start = 0;
volatile unsigned long hook_frame_end = 0;
volatile uint32_t ring_overflows = 0;

// --- TX queue entry type (missing in original file) ---
typedef struct {
  char target[8];
  uint8_t type;
} tx_entry_t;

// type: 0=INFO, 1=RT, 2=WT
static tx_entry_t tx_queue[8];
static uint8_t tx_q_head = 0;
static uint8_t tx_q_tail = 0;

static QueueHandle_t uart1_queue = NULL;
static TaskHandle_t uart1_task_handle = NULL;

// Push a byte into the ring buffer (ISR-safe). If buffer is full, increment overflow counter and drop the byte.
static inline void ring_push_byte(uint8_t b) {
  uint16_t next = ring_head + 1;
  if (next >= RING_SIZE) next = 0;
  if (next == ring_tail) {
    // buffer full
    ++ring_overflows;
    return;
  }
  ring_buf[ring_head] = b;
  ring_head = next;
}

// Minimal ISR hook for Serial1 RX - called from core ISR.
// Keep it very small: update volatile buffers & flags only.
void serial1_rx_hook(uint8_t c, unsigned long t) {
  // push byte to ring and mark frame boundaries
  ring_push_byte(c);

  if (!hook_receiving) {
    if (c == '$') {
      hook_receiving = 1;
      // start position is index of the '$' we just wrote: it's ring_head - 1 (wrap-aware)
      uint16_t pos = (ring_head == 0) ? (RING_SIZE - 1) : (ring_head - 1);
      hook_frame_start_pos = pos;
      hook_frame_start = t;
    }
  } else {
    if (c == ';') {
      // end position is index of the ';' we just wrote
      uint16_t pos = (ring_head == 0) ? (RING_SIZE - 1) : (ring_head - 1);
      hook_frame_end_pos = pos;
      hook_frame_end = t;
      hook_has_frame = 1;  // signal main loop
      hook_receiving = 0;
    }
  }
}

// Fast parser: parse integers from a frame like "$111 222 333;"
// $ => START FRAME
// AAAA BBBBB CCCCC DDDDD => CAP01 DATA(Inner CAPA)
// EEEE FFFFF GGGGG HHHHH => CAO02 DATA(External CAPA)
// IIII JJJJJ KKKKK LLLLL => MOC DATA
// ; => END FRAME
// Returns true if exactly FIELD_COUNT integers parsed and stores them into outValues.
static bool parse_values_fast(const char* frame, uint16_t* outValues, int expectedCount) {
  int idx = 0;
  const char* p = frame;
  if (*p == '$') ++p;

  while (*p && *p != ';' && idx < expectedCount) {
    while (*p == ' ') ++p;
    if (*p == '\0' || *p == ';') break;

    bool neg = false;
    if (*p == '-') {
      neg = true;
      ++p;
    }

    if (*p < '0' || *p > '9') return false;
    int v = 0;
    while (*p >= '0' && *p <= '9') {
      v = v * 10 + (*p - '0');
      ++p;
    }

    outValues[idx++] = (uint16_t)(neg ? -v : v);

    while (*p == ' ') ++p;
  }

  return (idx == expectedCount);
}

// ------------------ TX helper functions ------------------
#include <stdarg.h>

// --- UART TX helper uses uart_write_bytes on ESP32 to avoid Serial1/driver conflicts
static void uart_send_formatted(const char* fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  int len = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  if (len <= 0) return;
  if (len > (int)(sizeof(buf) - 1)) len = (int)(sizeof(buf) - 1);
#ifdef ARDUINO_ARCH_ESP32
  uart_write_bytes(SERIAL1_UART_NUM, buf, (size_t)len);
  uart_wait_tx_done(SERIAL1_UART_NUM, pdMS_TO_TICKS(100));
  Serial.print(F("[TX] "));
  Serial.println(buf);
#else
  Serial1.write((uint8_t*)buf, len);
  Serial.print(F("[TX] "));
  Serial.println(buf);
#endif
}
static void send_TOUCH() {
  uart_send_formatted("TOUCH");
  // expect plain OK (info)
  enqueue_tx(NULL, 0);
}

static void send_TOUCH_V() {
  uart_send_formatted("TOUCH+V");
  // expect version reply
  enqueue_tx(NULL, 0);
}

static void send_TOUCH_RT(const char* target) {
  // target expected like CAP01, CAP02, MOC01
  uart_send_formatted("TOUCH+RT+%s", target);
  // expect numeric OK <value>
  enqueue_tx(target, 1);
}

static void send_TOUCH_WT(const char* target, uint32_t value) {
  // format: TOUCH+WT+CAP01+"DATA" where DATA is decimal
  // clamp/validate value externally
  uart_send_formatted("TOUCH+WT+%s+%lu", target, (unsigned long)value);
  // expect numeric OK <value>
  enqueue_tx(target, 2);
}

// UART event task: reads events and bytes from the UART driver and pushes into ring buffer.
// This runs on a FreeRTOS task (interrupt-driven by the uart driver).
static void uart_event_task(void* pvParameters) {
  uart_event_t event;
  uint8_t dtmp[256];
  while (true) {
    if (xQueueReceive(uart1_queue, (void*)&event, portMAX_DELAY)) {
      if (event.type == UART_DATA) {
        int len = event.size;
        if (len > (int)sizeof(dtmp)) len = sizeof(dtmp);
        int r = uart_read_bytes(SERIAL1_UART_NUM, dtmp, len, 0);
        for (int i = 0; i < r; ++i) {
          uint8_t c = dtmp[i];
          // push into ring buffer
          ring_push_byte(c);
          // compute index of the byte we just wrote
          uint16_t pos = (ring_head == 0) ? (RING_SIZE - 1) : (ring_head - 1);

          // frame boundary detection (start '$', end ';')
          if (!hook_receiving) {
            if (c == '$') {
              hook_receiving = 1;
              hook_frame_start_pos = pos;
              hook_frame_start = micros();
            }
          } else {
            if (c == ';') {
              hook_frame_end_pos = pos;
              hook_frame_end = micros();
              hook_has_frame = 1;
              hook_receiving = 0;
            }
          }
        }
      } else if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) {
        // flush to recover
        uart_flush_input(SERIAL1_UART_NUM);
        // increment overflow counter (helps diagnostics)
        ++ring_overflows;
      } else {
        // ignore other events
      }
    }
  }
  vTaskDelete(NULL);
}

// Call this from setup() to initialize UART driver and start task
static void setup_uart1_driver() {
#ifdef ARDUINO_ARCH_ESP32
  // Configure and install UART driver for interrupt/event-driven RX
  uart_config_t uart_config = {
    .baud_rate = SERIAL_DEBUG_SPEED,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 0
  };
  uart_param_config(SERIAL1_UART_NUM, &uart_config);
  uart_set_pin(SERIAL1_UART_NUM, SERIAL1_TX_PIN, SERIAL1_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  // install driver with RX and non-zero TX buffer and event queue
  uart_driver_install(SERIAL1_UART_NUM, SERIAL1_RX_BUF_SIZE, SERIAL1_TX_BUF_SIZE, SERIAL1_EVENT_QUEUE_SZ, &uart1_queue, 0);

  // create uart event task
  xTaskCreatePinnedToCore(uart_event_task, "uart1_event_task", 4096, NULL, 12, &uart1_task_handle, 0);
#else
  // Fallback - non-ESP32 use Serial1.begin as before
#if defined(SERIAL1_RX_PIN) && defined(SERIAL1_TX_PIN)
  Serial1.begin(SERIAL_DEBUG_SPEED, SERIAL_8N1, SERIAL1_RX_PIN, SERIAL1_TX_PIN);
#else
  Serial1.begin(SERIAL_DEBUG_SPEED);
#endif
#endif
}

// ------------------ USB command parser (no String) ------------------
static void handle_usb_command(char* line) {
  // trim leading spaces
  while (*line == ' ') ++line;
  if (*line == '\0') return;

  // tokenise by space
  char* tok = strtok(line, " \t");
  if (!tok) return;

  // support forms like:
  // TOUCH
  // TOUCH+V
  // TOUCH+RT CAP01
  // TOUCH+RT+CAP01
  // TOUCH+WT CAP01 1234
  // TOUCH+WT+CAP01 1234

  if (strcmp(tok, "TOUCH") == 0) {
    send_TOUCH();
    return;
  }
  if (strcmp(tok, "TOUCH+V") == 0) {
    send_TOUCH_V();
    return;
  }
  // TOUCH+RT or TOUCH+RT+CAP01
  if (strncmp(tok, "TOUCH+RT", 8) == 0) {
    char target[16] = { 0 };
    if (tok[8] == '+') {
      // inline form TOUCH+RT+CAP01
      strncpy(target, tok + 9, sizeof(target) - 1);
      send_TOUCH_RT(target);
      return;
    }
    // separate arg
    char* arg = strtok(NULL, " \t");
    if (arg) {
      strncpy(target, arg, sizeof(target) - 1);
      send_TOUCH_RT(target);
      return;
    }
    Serial.println(F("Usage: TOUCH+RT <CAP01|CAP02|MOC01> or TOUCH+RT+CAP01"));
    return;
  }

  // TOUCH+WT (write) - needs target and data
  if (strncmp(tok, "TOUCH+WT", 8) == 0) {
    char target[16] = { 0 };
    char* valueStr = NULL;
    if (tok[8] == '+') {
      // inline form: TOUCH+WT+CAP01
      char* plus = strchr(tok + 9, '+');
      if (plus) {
        strncpy(target, tok + 9, (size_t)(plus - (tok + 9)));
        valueStr = plus + 1;
      } else {
        // maybe TOUCH+WT+CAP01 and next token is value
        strncpy(target, tok + 9, sizeof(target) - 1);
        valueStr = strtok(NULL, " \t");
      }
    } else {
      // separate args: TOUCH+WT CAP01 1234
      char* arg1 = strtok(NULL, " \t");
      char* arg2 = strtok(NULL, " \t");
      if (arg1) strncpy(target, arg1, sizeof(target) - 1);
      valueStr = arg2;
    }

    if (!target[0] || !valueStr) {
      Serial.println(F("Usage: TOUCH+WT <CAP01|CAP02|MOC01> <value 10..65535>"));
      return;
    }
    long v = strtol(valueStr, NULL, 10);
    if (v < 10 || v > 65535) {
      Serial.println(F("DATA out of range (10..65535)."));
      return;
    }
    send_TOUCH_WT(target, (uint32_t)v);
    return;
  }

  Serial.print(F("Unknown command: "));
  Serial.println(tok);
}

// ------------------ Incoming Serial1 command handling ------------------

static void enqueue_tx(const char* target, uint8_t type) {
  uint8_t next = (tx_q_tail + 1) & 7;
  if (next == tx_q_head) {
    // queue full, drop
    return;
  }
  tx_entry_t* e = &tx_queue[tx_q_tail];
  e->type = type;
  if (target) strncpy(e->target, target, sizeof(e->target) - 1);
  else e->target[0] = '\0';
  tx_q_tail = next;
}

static bool dequeue_tx(tx_entry_t* out) {
  if (tx_q_head == tx_q_tail) return false;
  *out = tx_queue[tx_q_head];
  tx_q_head = (tx_q_head + 1) & 7;
  return true;
}

static bool peek_tx(tx_entry_t* out) {
  if (tx_q_head == tx_q_tail) return false;
  *out = tx_queue[tx_q_head];
  return true;
}

// parse response lines coming back on Serial1 and associate with pending commands
static void parse_response_line(char* line) {
  // trim leading spaces
  while (*line == ' ') ++line;
  if (*line == '\0') return;

  // responses expected like:
  // OK
  // OK V1.00
  // OK 12345
  if (strncmp(line, "OK", 2) != 0) {
    // unknown response; echo
    Serial.print(F("[RX RESP] "));
    Serial.println(line);
    return;
  }
  char* p = line + 2;
  while (*p == ' ') ++p;

  tx_entry_t pending;
  bool hasPending = peek_tx(&pending);

  if (*p == '\0') {
    // plain OK
    Serial.println(F("[RX RESP] OK"));
    if (hasPending) dequeue_tx(&pending);
    return;
  }

  if (p[0] == 'V') {
    // version OK V1.00
    Serial.print(F("[RX RESP] "));
    Serial.println(p);
    if (hasPending) dequeue_tx(&pending);
    return;
  }

  // numeric payload
  long v = strtol(p, NULL, 10);
  Serial.print(F("[RX RESP] OK "));
  Serial.println(v);

  if (!hasPending) return;
  // consume pending and map to stored variables if target matches
  dequeue_tx(&pending);
}

// Try to read a newline-terminated line from the ring buffer when no framed frame is waiting.
// Returns true and fills outBuf if a full line was read (without trailing CR/LF).
static bool try_read_line_from_ring(char* outBuf, size_t maxLen) {
  if (hook_has_frame) return false;  // prefer frame handling

  noInterrupts();
  uint16_t head = ring_head;
  uint16_t tail = ring_tail;
  interrupts();

  if (head == tail) return false;  // no data

  size_t pos = 0;
  uint16_t idx = tail;
  bool found = false;

  // copy up to maxLen-1 bytes or until newline
  while (idx != head && pos < (maxLen - 1)) {
    uint8_t b = ring_buf[idx++];
    if (idx >= RING_SIZE) idx = 0;
    if (b == '\r') continue;
    if (b == '\n') {
      found = true;
      break;
    }
    outBuf[pos++] = (char)b;
  }

  if (!found) return false;

  outBuf[pos] = '\0';

  // advance ring_tail to after the newline we consumed
  noInterrupts();
  // move tail forward until after the newline
  while (ring_tail != head) {
    uint8_t b = ring_buf[ring_tail++];
    if (ring_tail >= RING_SIZE) ring_tail = 0;
    if (b == '\n') break;
  }
  interrupts();

  return true;
}


void setup() {

  // Lower global logging
  esp_log_level_set("*", ESP_LOG_ERROR);  // or ESP_LOG_WARN / ESP_LOG_INFO

  // Optionally : specifically quiet the RMT logs
  esp_log_level_set("esp32-hal-rmt", ESP_LOG_ERROR);
  Serial.begin(SERIAL_DEBUG_SPEED);
  // initialize UART driver / Serial1
  setup_uart1_driver();

  delay(500);
  // MOC deploy / retract user threshold settings
  userPotiDeploy.begin(ADC_USER_OPEN_THRESHOLD_PIN, NUM_SAMPLES, ADC_REF_VOLTAGE);

  //door handle actuator
  // actuator.begin(1, MOTOR1_ENABLE_PIN, MOTOR1_RPWM_PIN, MOTOR1_LPWM_PIN,
  //                DEPLOY_PWM, RETRACT_PWM,
  //                DEPLOY_TIME_MS, RETRACT_TIME_MS);

  // elacth Motor driver
  eLatchMotorDriver.begin(2, MOTOR2_ENABLE_PIN, MOTOR2_RPWM_PIN, MOTOR2_LPWM_PIN,
                          ELATCH_MOTOR_RUN_PWM, ELATCH_MOTOR_RUN_PWM,
                          ELATCH_MOTOR_RUN_TIME_CW, ELATCH_MOTOR_RUN_TIME_CCW, ELATCH_MOTOR_STOP_TIME);

  // Elatch switch configuration
  pinMode(E_LATCH_SW_PIN, INPUT_PULLUP);

  // Configure the PINS for usage
  pinMode(DEPLOY_SW_PIN, INPUT_PULLUP);
  pinMode(RETRACT_SW_PIN, INPUT_PULLUP);
  pinMode(OPEN_SWITCH_PIN, INPUT_PULLUP);
  // pinMode(DEPLOY_HANDLE_SW_PIN, INPUT_PULLUP);

  //led door handle - initialize FIRST, then wait before turning on
  ledCtrl.begin(LED_MAX_BRIGHTNESS);
  delay(100);  // Give LED strip time to initialize

  pinMode(ILLUMINATION_LED_PIN, OUTPUT);

  // Illumincation led glow
  digitalWrite(ILLUMINATION_LED_PIN, HIGH);
  // Door Handle Controller object configuration
  doorHandleController.setDependencies(&buttonDeploy, &buttonRetract, &(values[4]), &(values[0]), &ledCtrl, &eLatchMotorDriver);

  delay(1000);
  // Set the sensitivity for the CAPA sensors
  static char wtbuf1[64];
  sprintf(wtbuf1, "TOUCH+WT+CAP01+400");
  handle_usb_command(wtbuf1);

  delay(1000);

  static char wtbuf2[64];
  sprintf(wtbuf2, "TOUCH+WT+CAP02+500");
  handle_usb_command(wtbuf2);

  delay(1000);
  userPotiDeploy.update();
  // if (userPotiDeploy.hasNewAverage()) {
  uint32_t potValue = userPotiDeploy.getAverage();  // Avoid division by zero or out-of-range
  uint32_t scvalue = userPotiDeploy.getScaled(0, 4095, 10, 65535);

  // Serial.print(F("POTI_VALUE:"));
  // Serial.print(potValue);

  // Serial.print(F("\tSCALED_VALUE:"));
  // Serial.println(scvalue);
  static char wtbuf[64];
  sprintf(wtbuf, "TOUCH+WT+MOC01+%lu", scvalue);
  handle_usb_command(wtbuf);
  //   userPotiDeploy.setNewAverage(false);
  // }
  delay(10000);
  ledCtrl.ledOn(0, LedColor::WHITE);
  ledCtrl.ledOn(1, LedColor::WHITE);
  ledCtrl.updateLedState(0);  // Force immediate update
  ledCtrl.updateLedState(1);  // Force immediate update
  // Serial.println(F("Setup Completed."));
  // Serial.println(F(""));
}  // end setup

// === Main Loop ===
void loop(void) {
  unsigned long t_start = micros();
  unsigned long t_button = 0, t_latch = 0, t_moc = 0;
  unsigned long t_state = 0, t_motor = 0, t_led = 0;
  unsigned long t_end = 0;

  unsigned long t0 = micros();
  buttonDeploy.update();
  buttonRetract.update();
  // buttonDoorHandleDeploy.update();
  // buttonDoorHandleDeploy = digitalRead(DEPLOY_HANDLE_SW_PIN);
  buttonOpenRemoteSwitch.update();

  t_button = micros() - t0;

  t0 = micros();
  doorHandleController.updateeLatchSwitch();
  // doorHandleController.updateeDeploymentStatus();
  t_latch = micros() - t0;

  // t0 = micros();
  // // userPotiDeploy.update();
  // if (userPotiDeploy.hasNewAverage()) {
  //   uint32_t potValue = userPotiDeploy.getAverage();  // Avoid division by zero or out-of-range
  //   uint32_t scvalue = userPotiDeploy.getScaled(0, 4095, 10, 65535);

  //   Serial.print(F("POTI_VALUE:"));
  //   Serial.print(potValue);

  //   Serial.print(F("\tSCALED_VALUE:"));
  //   Serial.println(scvalue);
  //   static char wtbuf[64];
  //   sprintf(wtbuf, "TOUCH+WT+MOC01+%lu", scvalue);
  //   handle_usb_command(wtbuf);
  //   userPotiDeploy.setNewAverage(false);
  // }
  // t_poti = micros() - t0;

  t0 = micros();
  moc_reading();
  if ((values[8] == 6500) || buttonOpenRemoteSwitch.getswitchStatus())
    doorHandleController.setState(DOOR_HANDLE_OPEN);
  t_moc = micros() - t0;

  t0 = micros();
  doorHandleController.refreshState();
  t_state = micros() - t0;

  // t0 = micros();
  // actuator.update();
  // t_actuator = micros() - t0;

  t0 = micros();
  eLatchMotorDriver.update();
  t_motor = micros() - t0;

  t0 = micros();
  //   for (uint16_t i = 0; i < NUM_PIXELS; ++i)
  //     ledCtrl.updateLedState(i);

  if (doorHandleController.getswitchStatus())
    ledCtrl.ledOn(5, LedColor::WHITE);
  else
    ledCtrl.ledOff(5);

  if (values[0] == 6500)
    ledCtrl.ledOn(3, LedColor::WHITE);
  else ledCtrl.ledOff(3);

  if (values[4] == 6500)
    ledCtrl.ledOn(2, LedColor::WHITE);
  else
    ledCtrl.ledOff(2);

  if (values[8] == 6500)
    ledCtrl.ledOn(4, LedColor::WHITE);
  else
    ledCtrl.ledOff(4);

  ledCtrl.updateLedState();

  t_led = micros() - t0;

  t_end = micros();

#if DEBUGGING_ENABLED
  // if ((t_end - t_start) > 500) {
  // Serial.print("Timing(us): ");
  Serial.print("Buttons:");
  Serial.print(t_button);
  Serial.print("\t LatchSw:");
  Serial.print(t_latch);
  // Serial.print("\t Potis:");
  // Serial.print(t_poti);
  Serial.print("\t MOC:");
  Serial.print(t_moc);
  Serial.print("\t HandleState:");
  Serial.print(t_state);
  Serial.print("\t Motor:");
  Serial.print(t_motor);
  // Serial.print("\t Actuator:");
  // Serial.print(t_actuator);
  Serial.print("\t LED:");
  Serial.print(t_led);
  Serial.print("\t Total:");
  Serial.println(t_end - t_start);
  // }
// #else
//   Serial.print("MOC_RAW:");
//   Serial.print(values[9]);
//   Serial.print("\tMOC_THR:");
//   Serial.print(values[10]);
//   Serial.print("\tMOC_SIG:");
//   Serial.print(values[8]);

//   Serial.print("\tINN_RAW:");
//   Serial.print(values[1]);
//   Serial.print("\tINN_THR:");
//   Serial.print(values[2]);
//   Serial.print("\tINN_SIG:");
//   Serial.print(values[0]);

//   Serial.print("\tEXT_RAW:");
//   Serial.print(values[5]);
//   Serial.print("\tEXT_THR:");
//   Serial.print(values[6]);
//   Serial.print("\tEXT_SIG:");
//   Serial.println(values[4]);
#endif

}  //end main loop



// Replace moc_reading() with the version below — reads USB console and processes frames
void moc_reading() {
  // Read USB-Serial input (non-blocking) and handle commands
  static char lineBuf[64];
  static uint8_t linePos = 0;
  while (Serial.available() > 0) {
    int c = Serial.read();
    if (c <= 0) break;
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuf[linePos] = '\0';
      if (linePos > 0) {
        handle_usb_command(lineBuf);
      }
      linePos = 0;
    } else {
      if (linePos < (sizeof(lineBuf) - 1)) {
        lineBuf[++linePos] = (char)c;
      }
    }
  }

  // Try to read a newline-terminated line from the ring buffer (responses coming from UART)
  char uartLine[80];
  if (try_read_line_from_ring(uartLine, sizeof(uartLine))) {
    parse_response_line(uartLine);
  }

  // If a framed MOC packet was captured by the UART task, copy and parse it here
  if (hook_has_frame) {
    noInterrupts();
    uint16_t start = hook_frame_start_pos;
    uint16_t end = hook_frame_end_pos;
    // compute length (inclusive), handling wrap
    uint32_t len32;
    if (end >= start) {
      len32 = (uint32_t)(end - start) + 1;
    } else {
      len32 = (uint32_t)(RING_SIZE - start) + (uint32_t)(end) + 1;
    }
    if (len32 >= (uint32_t)maxFrameLength) len32 = maxFrameLength - 1;
    uint16_t len = (uint16_t)len32;

    // copy out of ring with wrap handling
    for (uint16_t i = 0; i < len; ++i) {
      uint16_t idx = start + i;
      if (idx >= RING_SIZE) idx -= RING_SIZE;
      rawFrame[i] = (char)ring_buf[idx];
    }
    rawFrame[len] = '\0';

    // advance tail past the consumed frame (end + 1) mod RING_SIZE
    uint16_t newTail = end + 1;
    if (newTail >= RING_SIZE) newTail = 0;
    ring_tail = newTail;

    // copy timestamps and clear flag
    frameStartMicros = hook_frame_start;
    frameEndMicros = hook_frame_end;
    hook_has_frame = 0;
    interrupts();

    bool ok = parse_values_fast((const char*)rawFrame, values, FIELD_COUNT);

    if (verbosePrint) {
      Serial.print(F("[RawFrame] "));
      Serial.println(rawFrame);
    }
    if (frameTimingEnabled) {
      unsigned long procLatency = micros() - frameEndMicros;
      Serial.print(F("IN_CAPA_EN:"));
      Serial.print(values[0] + 6000);
      Serial.print(F("\tIN_CAPA_V:"));
      Serial.print(values[1]);
      Serial.print(F("\tIN_CAPA_TH:"));
      Serial.print(values[2]);
      Serial.print(F("\tE_CAPA_EN:"));
      Serial.print(values[4] + 6200);
      // Serial.print(F("\tEXT_CAPA_V:"));
      // Serial.print(values[5]);
      // Serial.print(F("\tEXT_CAPA_TH:"));
      // Serial.print(values[6]);
      Serial.print(F("\tMOC_EN:"));
      Serial.print(values[8]);
      Serial.print(F("\tMOC_V:"));
      Serial.print(values[9]);
      Serial.print(F("\tMOC_TH:"));
      Serial.println(values[10]);
      // Serial.print(F("\tPROCESSTIME:"));
      // Serial.println(procLatency);
    }
    if (verbosePrint) {
      if (ok) {
        for (int i = 0; i < FIELD_COUNT; ++i) {
          Serial.print(F("F"));
          Serial.print(i + 1);
          Serial.print(F(":"));
          Serial.print(values[i]);
          if (i + 1 < FIELD_COUNT) Serial.print(F(" "));
        }
        Serial.println();
      } else {
        Serial.println(F("Parsed: invalid format"));
      }
    }
    lastFrameEndMicros = frameEndMicros;
  }
}