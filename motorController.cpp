#include "MotorController.h"
#include "CapaTouchSensor.h"


MotorController actuator;


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

  pinMode(enablePin, OUTPUT);
  pinMode(rpwmPin, OUTPUT);
  pinMode(lpwmPin, OUTPUT);
  digitalWrite(enablePin, LOW);
  setupHighFreqPWM();
  stop();

  Serial.println(F("Actuator driver initialization completed"));
}

void MotorController::update() {
  switch (state) {
    case MOTOR_STOP:
      interlockState = MOTOR_NOT_BLOCKED;
      break;

    case MOTOR_START_DEPLOY:
      // logger.verbose("Motor will run in deploy for time: " + String(deployDuration));
      // logger.verbose("Motor will run in deploy at speed of: " + String(deployPwm));

      enable(true);
      driveCW();
      startTime = millis();
      state = MOTOR_RUNNING;
      commandState = MOTOR_START_DEPLOY;
      interlockState = MOTOR_BLOCKED;
      Serial.println(F("Actuator Deploying."));
      break;

    case MOTOR_START_RETRACT:
      // logger.verbose("Motor will run in retract for time: " + String(retractDuration));
      // logger.verbose("Motor will run in retract at speed of: " + String(retractPwm));
      enable(true);
      driveCCW();
      startTime = millis();
      state = MOTOR_RUNNING;
      commandState = MOTOR_START_RETRACT;
      interlockState = MOTOR_BLOCKED;
      Serial.println(F("Actuator Retracting."));
      break;

    case MOTOR_RUNNING:
      if ((millis() - startTime >= deployDuration && commandState == MOTOR_START_DEPLOY) || (millis() - startTime >= retractDuration && commandState == MOTOR_START_RETRACT)) {
        state = MOTOR_STOPPING;
        // Serial.println(F("Motor run complete."));
      }
      break;

    case MOTOR_STOPPING:
      stop();
      enable(false);
      state = MOTOR_STOP;
      commandState = MOTOR_STOP;
      Serial.println(F("Actuator stopped."));
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

void MotorController::driveCW() {
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
