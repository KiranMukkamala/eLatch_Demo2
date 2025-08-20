#ifndef TIMEOUTMANAGER_H
#define TIMEOUTMANAGER_H

#include <Arduino.h>

#define MAX_TIMEOUTS 5

// Timeout IDs
#define TIMEOUT_DEPLOYED_DOOR_HANDLE 0
//#define TIMEOUT_MOC_CONNECTION 1
#define TIMEOUT_SERIAL_CONNECTION 2
#define TIMEOUT_HANDLE_DEPLOY_DELAY 3
#define TIMEOUT_E_LATCH 5


// Default timeout durations (optional)
#define TIMEOUT_HANDLE_DEPLOY_DELAY_MS 200
#define TIMEOUT_DEPLOYED_DOOR_HANDLE_MS 600000
#define TIMEOUT_CONNECTION_SERIAL_TIMEOUT_MS 100
#define TIMEOUT_E_LATCH_COOLDOWN_TIME_MS  1000


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

#endif // TIMEOUT_MANAGER_H
