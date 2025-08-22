#ifndef MOCREADER_H
#define MOCREADER_H

#include <Arduino.h>
#include "ADCReader.h"
#include "global.h"
#include "constants.h"


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
