#include "motorController.h"

// MotorController actuator; // define the extern instance

void MotorController::begin(uint8_t en, uint8_t r, uint8_t l,
                            uint16_t deployPwm_, uint16_t retractPwm_,
                            unsigned long deployDur, unsigned long retractDur) {
  enablePin = en;
  rpwmPin = r;
  lpwmPin = l;
  deployPwm = deployPwm_;
  retractPwm = retractPwm_;
  deployDuration = deployDur;
  retractDuration = retractDur;
  commandState = MOTOR_STOP;
  prevCommandDirection = MOTOR_STOP;

  pinMode(enablePin, OUTPUT);
  pinMode(rpwmPin, OUTPUT);
  pinMode(lpwmPin, OUTPUT);
  digitalWrite(enablePin, LOW);
  setupHighFreqPWM();
  stop();

  Serial.println("Actuator driver initialization completed");
}

MotorState MotorController::getRecentCommand() const {
  return prevCommandDirection;
}

void MotorController::update() {
  switch (state) {
    case MOTOR_STOP:
      interlockState = MOTOR_NOT_BLOCKED;
      break;

    case MOTOR_START_DEPLOY:
      enable(true);
      driveCW();
      startTime = millis();
      state = MOTOR_RUNNING;
      commandState = MOTOR_START_DEPLOY;
      prevCommandDirection = MOTOR_START_DEPLOY;
      interlockState = MOTOR_BLOCKED;
      Serial.println("Motor Running CW.");
      break;

    case MOTOR_START_RETRACT:
      enable(true);
      driveCCW();
      startTime = millis();
      state = MOTOR_RUNNING;
      commandState = MOTOR_START_RETRACT;
      prevCommandDirection = MOTOR_START_RETRACT;
      interlockState = MOTOR_BLOCKED;
      Serial.println("Motor Running CCW.");
      break;

    case MOTOR_RUNNING:
      if ((millis() - startTime >= deployDuration && commandState == MOTOR_START_DEPLOY) || (millis() - startTime >= retractDuration && commandState == MOTOR_START_RETRACT)) {
        state = MOTOR_STOPPING;
        Serial.println("Motor run complete.");
      }
      break;

    case MOTOR_STOPPING:
      stop();
      enable(false);
      state = MOTOR_STOP;
      commandState = MOTOR_STOP;
      Serial.println(F("Motor stopped."));
      break;
  }
}

void MotorController::triggerAction(int16_t value) {
  if (state == MOTOR_STOP) {
    if (value == 4000) {
      state = MOTOR_START_DEPLOY;
    } else if (value == 3000) {
      state = MOTOR_START_RETRACT;
    }
  }
}

void MotorController::enable(bool on) {
  digitalWrite(enablePin, on ? HIGH : LOW);
}

#ifdef ARDUINO_ARCH_ESP32

void MotorController::driveCW() {
  // RPWM drives deploy; LPWM off
  uint32_t out = scaleToResolution(deployPwm);
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)rpwmLedcChannel, out);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)rpwmLedcChannel);

  ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)lpwmLedcChannel, 0);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)lpwmLedcChannel);
}

void MotorController::driveCCW() {
  // LPWM drives retract; RPWM off
  uint32_t out = scaleToResolution(retractPwm);
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)lpwmLedcChannel, out);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)lpwmLedcChannel);

  ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)rpwmLedcChannel, 0);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)rpwmLedcChannel);
}

void MotorController::stop() {
  // set LEDC duties to zero and ensure pins low
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)rpwmLedcChannel, 0);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)rpwmLedcChannel);
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)lpwmLedcChannel, 0);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)lpwmLedcChannel);

  digitalWrite(rpwmPin, LOW);
  digitalWrite(lpwmPin, LOW);
}

void MotorController::setupHighFreqPWM() {
  // configure LEDC timer (use TIMER_0) and attach channels to pins
  ledc_timer_config_t ledc_timer;
  memset(&ledc_timer, 0, sizeof(ledc_timer));
  ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;
  ledc_timer.duty_resolution = (ledc_timer_bit_t)pwmResBits;
  ledc_timer.freq_hz = pwmFreqHz;
  ledc_timer.timer_num = LEDC_TIMER_0;
  ledc_timer_config(&ledc_timer);

  // configure RPWM channel
  ledc_channel_config_t ch_r = {};
  ch_r.gpio_num = rpwmPin;
  ch_r.speed_mode = LEDC_HIGH_SPEED_MODE;
  ch_r.channel = (ledc_channel_t)rpwmLedcChannel;
  ch_r.intr_type = LEDC_INTR_DISABLE;
  ch_r.timer_sel = LEDC_TIMER_0;
  ch_r.duty = 0;
  ledc_channel_config(&ch_r);

  // configure LPWM channel
  ledc_channel_config_t ch_l = {};
  ch_l.gpio_num = lpwmPin;
  ch_l.speed_mode = LEDC_HIGH_SPEED_MODE;
  ch_l.channel = (ledc_channel_t)lpwmLedcChannel;
  ch_l.intr_type = LEDC_INTR_DISABLE;
  ch_l.timer_sel = LEDC_TIMER_0;
  ch_l.duty = 0;
  ledc_channel_config(&ch_l);

  // initialize outputs to 0
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)rpwmLedcChannel, 0);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)rpwmLedcChannel);
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)lpwmLedcChannel, 0);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)lpwmLedcChannel);
}

#else // AVR / legacy code (keeps original timer usage)

void MotorController::driveCW() {
  // original AVR Timer1 PWM control (keeps compatibility for AVR)
  TCCR1A &= ~(1 << COM1B1);
  OCR1B = 0;
  digitalWrite(lpwmPin, LOW);
  OCR1A = deployPwm;
  TCCR1A |= (1 << COM1A1);
}

void MotorController::driveCCW() {
  TCCR1A &= ~(1 << COM1A1);
  OCR1A = 0;
  digitalWrite(rpwmPin, LOW);
  OCR1B = retractPwm;
  TCCR1A |= (1 << COM1B1);
}

void MotorController::stop() {
  TCCR1A &= ~((1 << COM1A1) | (1 << COM1B1));
  OCR1A = 0;
  OCR1B = 0;
  digitalWrite(rpwmPin, LOW);
  digitalWrite(lpwmPin, LOW);
}

void MotorController::setupHighFreqPWM() {
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  TCCR1A |= (1 << WGM11) | (1 << WGM10);
  TCCR1B |= (1 << WGM12);
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1);
  TCCR1B |= (1 << CS10);  // No prescaler
}

#endif // ARDUINO_ARCH_ESP32

MotorState MotorController::getState() const {
  return state;
}

bool MotorController::setState(MotorState value) {
  if (interlockState != MOTOR_BLOCKED) {
    // Serial.println(String(F("Motor is unblocked, setState changing to ")) + String(value));
    state = value;
    return true;
  }
  return false;
}
