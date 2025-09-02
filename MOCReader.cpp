#include "MOCReader.h"
#include "TimeoutManager.h"
//#include "DebugLogger.h"

/*==== Class Initialization ===*/
MOCReader mocReader;


MOCReader::MOCReader()
  : packetIndex(0),
    packetStarted(false),
    sampleIndex(0),
    directionInverted(true),
    prevUpperTrig(false),
    prevLowerTrig(false),
    triggerActive(false),
    triggerType(NO_TRIGGER),
    onSignificantChange(nullptr) {
  memset(packetBuffer, 0, PACKET_BUFFER_SIZE);
  memset(samples, 0, sizeof(samples));
}

void MOCReader::begin() {
  Serial1.begin(MOC_COM_SPEED);
  Serial.println(F("MOC driver initialization completed"));
}

void MOCReader::update() {
  readMOCPacket();
}

void MOCReader::readMOCPacket() {
  const int MAX_BYTES_PER_CALL = 8; // Lower for better responsiveness
  int bytesProcessed = 0;
  while (Serial1.available() && bytesProcessed < MAX_BYTES_PER_CALL) {
    int8_t c = Serial1.read();
    bytesProcessed++;

    if (c == '$') {
      packetStarted = true;
      packetIndex = 0;
      timeout.cancel(TIMEOUT_MOC_CONNECTION);
    } else if (packetStarted) {
      if (c == ';') {
        packetBuffer[packetIndex] = '\0';
        processPacket(packetBuffer);
        packetStarted = false;
      } else {
        if (packetIndex < PACKET_BUFFER_SIZE - 1) {
          packetBuffer[packetIndex++] = c;
        } else {
          packetStarted = false;
          packetIndex = 0;
         // Serial.println(F("Packet overflow. Packet discarded."));
        }
      }
    }
  }

  if (!timeout.isActive(TIMEOUT_MOC_CONNECTION)) {
    timeout.start(TIMEOUT_MOC_CONNECTION, CONNECTION_MOC_TIMEOUT_MS);
  }

  if (timeout.isElapsed(TIMEOUT_MOC_CONNECTION)) {
   // Serial.println(F("MOC Serial Connection Lost!"));
    timeout.cancel(TIMEOUT_MOC_CONNECTION);
  }
}

void MOCReader::processPacket(char* packet) {
  int16_t dataFrame[5];
  int8_t frameSize = sscanf(packet, "%d %d %d %d %d",
                            &dataFrame[0], &dataFrame[1], &dataFrame[2], &dataFrame[3], &dataFrame[4]);
  // Serial.println(" Received Packet:: " + String(dataFrame[1]));

  if (frameSize == 5) {
    int16_t reference = dataFrame[0];
    int16_t value = dataFrame[1];
    processMOCData(reference, value);
  } else {
   // Serial.println(F("Malformed packet received."));
  }
}

// Refactored: only collects data, all logic is handled in the callback!
void MOCReader::processMOCData(int16_t reference, int16_t value) {
  static bool bufferFilled = false;
  int oldest = 0, mid = 0, newest = 0;
  int oldestIdx = 0, midIdx = 0, newestIdx = 0;

  // Store sample in ring buffer
  samples[sampleIndex] = value;
  sampleIndex = static_cast<uint8_t>((static_cast<uint16_t>(sampleIndex) + 1U) % SAMPLE_COUNT);

  if (sampleIndex == 0U) bufferFilled = true;

  if (bufferFilled) {
    oldestIdx = static_cast<int16_t>((static_cast<uint16_t>(sampleIndex)) % SAMPLE_COUNT);
    midIdx = static_cast<int16_t>((static_cast<uint16_t>(sampleIndex) + 1U) % SAMPLE_COUNT);
    newestIdx = static_cast<int16_t>((static_cast<uint16_t>(sampleIndex) + 2U) % SAMPLE_COUNT);

    oldest = samples[oldestIdx];
    mid = samples[midIdx];
    newest = samples[newestIdx];
  }

  int16_t diff = static_cast<int16_t>(value) - static_cast<int16_t>(reference);
  int16_t unlockAnalogInput = userPotiDeploy.getAverage();
  int16_t openAnalogInput = userPotiRetract.getAverage();
  int16_t unlockThreshold = userUnlockThreshold(reference, unlockAnalogInput);
  // int16_t openThreshold = userOpenThreshold(reference, openAnalogInput);


  // Populate struct and call callback
  if (onSignificantChange != nullptr) {
    MOCSignalData data = {
      reference,
      value,
      diff,
      unlockAnalogInput,
      openAnalogInput,
      unlockThreshold,
      // openThreshold,
      oldest,
      mid,
      newest
    };
    onSignificantChange(data);
  }
}

int16_t MOCReader::userUnlockThreshold(int16_t baseValue, int16_t analogInput) {
  int16_t offset = map(analogInput, 0, 1023, MOC_UNLOCK_THRESHOLD_MIN, MOC_UNLOCK_THRESHOLD_MAX);
  return abs(offset);
}

int16_t MOCReader::userOpenThreshold(int16_t baseValue, int16_t analogInput) {
  int16_t offset = map(analogInput, 0, 1023, MOC_OPEN_THRESHOLD_MIN, MOC_OPEN_THRESHOLD_MAX);
  return abs(offset);
}

void MOCReader::setChangeCallback(CallbackType cb) {
  onSignificantChange = cb;
}

void MOCReader::setDirectionInverted(bool inverted) {
  directionInverted = inverted;
}


// MOCReader mocReader;
