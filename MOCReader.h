#ifndef MOCREADER_H
#define MOCREADER_H

#include <Arduino.h>
#include "ADCReader.h"
#include "global.h"

#define MOC_COM_SPEED 19200
#define CONNECTION_MOC_TIMEOUT_MS 500
#define TIMEOUT_MOC_CONNECTION 1

#define PACKET_BUFFER_SIZE 64
#define SAMPLE_COUNT 1


#define MOC_UNLOCK_THRESHOLD_MIN 350
#define MOC_UNLOCK_THRESHOLD_MAX 1650
#define MOC_OPEN_THRESHOLD_MIN 2101
#define MOC_OPEN_THRESHOLD_MAX 5000


// Structure to hold all relevant MOC data for the callback
struct MOCSignalData {
    int16_t reference;
    int16_t value;
    int16_t diff;
    int16_t unlockAnalogInput;
    int16_t openAnalogInput;
    int16_t unlockThreshold;
    int16_t openThreshold;
    int16_t oldest;
    int16_t mid;
    int16_t newest;
};

class MOCReader {
public:
    MOCReader();

    void begin();
    void update();

    using CallbackType = void (*)(const MOCSignalData&);
    void setChangeCallback(CallbackType cb);
    void setDirectionInverted(bool inverted);           //not used

    // --- User threshold helpers ---
    int16_t userUnlockThreshold(int16_t baseValue, int16_t analogInput);
    int16_t userOpenThreshold(int16_t baseValue, int16_t analogInput);

private:
    void readMOCPacket();
    void processPacket(char* packet);
    void processMOCData(int16_t reference, int16_t value);

    char packetBuffer[PACKET_BUFFER_SIZE];
    uint8_t packetIndex;
    bool packetStarted;

    int16_t samples[SAMPLE_COUNT];
    uint8_t sampleIndex;

    bool directionInverted;
    bool prevUpperTrig;
    bool prevLowerTrig;

    bool triggerActive;
    enum { NO_TRIGGER, UPPER_TRIGGER, LOWER_TRIGGER } triggerType;

    CallbackType onSignificantChange;
};

extern MOCReader mocReader;

#endif // MOCREADER_H
