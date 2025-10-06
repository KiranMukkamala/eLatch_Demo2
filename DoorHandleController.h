#include <stdint.h>
#ifndef DOOR_HANDLE_CONTROLLER_H
#define DOOR_HANDLE_CONTROLLER_H

#include <Arduino.h>
#include "constants.h"
#include "Debounce.h"
// #include "CapaTouchSensor.h"
#include "LEDControl.h"
#include "motorController.h"
#include "RelayController.h"

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

class DoorHandleController {
public:
  DoorHandleController();
  void refreshState();
  void setState(DoorHandleState state);
  void Check_Disable_Locking();
  DoorHandleState getState() const;
  void setDependencies(Debounce* deployBtn, Debounce* retractBtn, Debounce* deployHandleBtn,
                       uint16_t* extCapa, uint16_t* inrCapa,
                       LEDControl* ledCtrl, MotorController* actuator,
                       RelayController* eLatchMotorDriver);
  void updateeLatchSwitch(void);
  bool getswitchStatus(void);
private:
  DoorHandleState doorHandleState;
  Debounce* buttonDeploy;
  Debounce* buttonRetract;
  Debounce* buttonHandleDeploy;
  uint16_t* extcapaSensor;
  uint16_t* inrcapaSensor;
  LEDControl* ledCtrl;
  MotorController* actuator;
  RelayController* eLatchMotorDriver;
  bool Disable_Locking = false;  // for disabling the external capa sensor
  uint16_t Nb_Open_Attempt;
  //eLatch Switch status
  bool latchSwitchState = false;
  bool risingSwEdge = false;
  bool fallingSwEdge = false;
  bool currentSwState = HIGH;
  bool lastSwState = HIGH;
};

#endif  // DOOR_HANDLE_CONTROLLER_H
