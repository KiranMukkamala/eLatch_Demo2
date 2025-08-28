#ifndef TIMEOUTMANAGER_H
#define TIMEOUTMANAGER_H

#include <Arduino.h>
#include "constants.h"


class TimeoutManager {
public:
  TimeoutManager();

  void start(uint8_t id, unsigned long durationMs);
  bool isElapsed(uint8_t id);
  void cancel(uint8_t id);
  bool isActive(uint8_t id);

private:
  struct Timeout {
    bool active;
    unsigned long startTime;
    unsigned long duration;
  };
  Timeout timeouts[MAX_TIMEOUTS];
};

extern TimeoutManager timeout;

#endif  // TIMEOUT_MANAGER_H
