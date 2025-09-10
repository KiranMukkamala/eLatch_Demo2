# eLatch_Demo2 Instructions

## Project Architecture
- This is an Arduino-based firmware project for an electronic latch demo (Audi FDH), integrating capacitive (CAPA) and MOC sensors, motor/relay control, and LED feedback.
- The main entry point is `eLatch_Demo2.ino`, which orchestrates hardware initialization, sensor polling, state machine logic, and actuator control.
- Core logic is modularized into C++ classes:
  - `DoorHandleController`: Manages door handle state transitions and encapsulates the main state machine.
  - `Debounce`, `CapaTouchSensor`, `LEDControl`, `motorController`, `RelayController`, `MOCReader`: Each handles a specific hardware or logic subsystem.
- Data flows from sensors (MOC, CAPA) through state logic to actuators (motor, relay) and feedback (LEDs).

## Developer Workflows
- **Build/Upload:** Use the Arduino extension in VS Code. The board is `arduino:avr:micro` and the main sketch is `eLatch_Demo2.ino`.
- **Debugging:** Serial output is routed via `Serial1` at 19200 baud. Timing and state info is printed conditionally using the `DEBUGGING_ENABLED` macro.
- **IntelliSense:** Ensure `.vscode/c_cpp_properties.json` includes Arduino core and variant paths for code completion.
- **Configuration:** `.vscode/arduino.json` and `.vscode/c_cpp_properties.json` must match the board and sketch file.

## Project-Specific Patterns
- All hardware logic is abstracted into classes. The main `.ino` file only wires up dependencies and calls high-level methods.
- State transitions are managed via `DoorHandleController.setState()` and `refreshState()`. Avoid direct manipulation of state variables outside this class.
- Sensor and actuator objects are instantiated globally and injected into controllers via `setDependencies()`.
- Serial output for debugging uses `Serial1.print` and is gated by `DEBUGGING_ENABLED`.
- Timing analysis is performed in the main loop and output as a single line per iteration.

## Integration Points
- No external libraries beyond Arduino core are required; all logic is custom and local.
- Hardware pin assignments and configuration are documented in comments at the top of `eLatch_Demo2.ino`.
- All cross-component communication is via method calls and dependency injection, not global variables.

## Key Files
- `eLatch_Demo2.ino`: Main application logic, setup, and loop.
- `DoorHandleController.h/cpp`: State machine and door handle logic.
- `MOCReader.h/cpp`, `CapaTouchSensor.h/cpp`, `LEDControl.h/cpp`, `motorController.h/cpp`, `RelayController.h/cpp`, `Debounce.h/cpp`: Subsystem modules.
- `.vscode/arduino.json`, `.vscode/c_cpp_properties.json`: VS Code/Arduino configuration.

## Example Patterns
```cpp
// State transition
if (event) {
    doorHandleController.setState(DOOR_HANDLE_OPEN);
}
// Timing debug output
#if DEBUGGING_ENABLED
    Serial1.print("Timing(us): ...");
#endif
```

---
If any section is unclear or missing, please provide feedback for further refinement.
