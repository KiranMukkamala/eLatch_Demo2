#include "TimeoutManager.h"

/*==== Class Initialization ===*/
TimeoutManager timeout;

TimeoutManager::TimeoutManager() {
    for (int i = 0; i < MAX_TIMEOUTS; ++i) {
        timeouts[i].active = false;
        timeouts[i].startTime = 0;
        timeouts[i].duration = 0;
    }
}

void TimeoutManager::start(uint8_t id, unsigned long durationMs) {
    if (id >= MAX_TIMEOUTS) return;
    timeouts[id].active = true;
    timeouts[id].startTime = millis();
    timeouts[id].duration = durationMs;
}

bool TimeoutManager::isElapsed(uint8_t id) {
    if (id >= MAX_TIMEOUTS || !timeouts[id].active) return false;
    return (millis() - timeouts[id].startTime >= timeouts[id].duration);
}

void TimeoutManager::cancel(uint8_t id) {
    if (id >= MAX_TIMEOUTS) return;
    timeouts[id].active = false;
}

bool TimeoutManager::isActive(uint8_t id) {
    if (id >= MAX_TIMEOUTS) return false;
    return timeouts[id].active;
}


