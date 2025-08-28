/*
  Iridium Module

  This module configures and manages the RockBLOCK 9603 Iridium modem.
  It provides functions to set up the modem, build and transmit SBD messages,
  handle incoming MT messages, and manage power states. Callbacks for the
  IridiumSBD library are included for debugging and non-blocking transmission
  or GNSS acquisition.

  Ring Buffer Design:
  - Store messages in a RAM ring buffer (oldest-first).
  - Each sample enqueues one SBD record (SBD_MSG_SIZE bytes).
  - Send policy is selectable at runtime:
      * OLDEST_FIRST: transmit up to SBD_MAX_MSGS oldest messages
      * NEWEST_FIRST: transmit up to SBD_MAX_MSGS newest messages
  - On success, remove exactly the number sent from the corresponding end.
  - On failure, keep all queued messages.
*/

// ----------------------------------------------------------------------------
// Configuration knobs
// ----------------------------------------------------------------------------

// Selectable TX policy
enum class SendPolicy : uint8_t { OLDEST_FIRST = 0,
                                  NEWEST_FIRST = 1 };
// Change default desired:
volatile SendPolicy sendPolicy = SendPolicy::NEWEST_FIRST;  // Runtime policy

#ifndef RING_BUFFER_CAPACITY
// Max messages to hold in RAM beyond a single 340-byte Iridium window.
// memory usage = RING_BUFFER_CAPACITY * SBD_MSG_SIZE bytes.
#define RING_BUFFER_CAPACITY 100
#endif

// ----------------------------------------------------------------------------
// Flat storage and indices
// ----------------------------------------------------------------------------
static uint8_t ringStorage[RING_BUFFER_CAPACITY * SBD_MSG_SIZE];

static uint16_t ringHead = 0;     // Index of oldest message (0..capacity-1)
static uint16_t ringTail = 0;     // Index to write the next message
static uint16_t ringCount = 0;    // Number of valid messages in the ring
static uint32_t ringDropped = 0;  // How many oldest records we overwrote

// ----------------------------------------------------------------------------
// Ring buffer utilities
// ----------------------------------------------------------------------------
uint16_t ringBufferCapacity() {
  return (uint16_t)RING_BUFFER_CAPACITY;
}
uint16_t ringBufferSize() {
  return ringCount;
}
bool ringBufferEmpty() {
  return ringCount == 0;
}
bool ringBufferFull() {
  return ringCount >= RING_BUFFER_CAPACITY;
}
uint32_t ringBufferDropped() {
  return ringDropped;
}

void ringBufferClear() {
  ringHead = 0;
  ringTail = 0;
  ringCount = 0;
  // Do not reset ringDropped (useful telemetry)
}

// Returns pointer to the start of a slot i (0..capacity-1) in the flat array
static uint8_t* ringSlotPtr(uint16_t slotIndex) {
  return &ringStorage[(size_t)slotIndex * (size_t)SBD_MSG_SIZE];
}

// Push one message (SBD_MSG_SIZE bytes) at tail.
// Policy: if full, overwrite oldest (advance head) and count a drop.
void ringBufferPush(const uint8_t* bytes) {
  if (ringBufferFull()) {
    ringHead = (uint16_t)((ringHead + 1) % RING_BUFFER_CAPACITY);
    ringDropped++;
    // ringCount stays at capacity
  } else {
    ringCount++;
  }
  memcpy(ringSlotPtr(ringTail), bytes, SBD_MSG_SIZE);
  ringTail = (uint16_t)((ringTail + 1) % RING_BUFFER_CAPACITY);
}

// Peek message at offset i from the oldest (0 = oldest). Returns false if OOR.
bool ringBufferPeek(uint16_t i, uint8_t* dst) {
  if (i >= ringCount) return false;
  uint16_t slot = (uint16_t)((ringHead + i) % RING_BUFFER_CAPACITY);
  memcpy(dst, ringSlotPtr(slot), SBD_MSG_SIZE);
  return true;
}

// Pop n oldest messages (remove them). Safe if n > size (clamps to clear).
void ringBufferPop(uint16_t n) {
  if (n >= ringCount) {
    ringBufferClear();
    return;
  }
  ringHead = (uint16_t)((ringHead + n) % RING_BUFFER_CAPACITY);
  ringCount -= n;
}

// ----------------------------------------------------------------------------
// Tail-based helpers for newest-first policy
// ----------------------------------------------------------------------------

// Peek message at offset i from the newest (0 = newest). Returns false if OOR.
bool ringBufferPeekNewest(uint16_t i, uint8_t* dst) {  // NEW
  if (i >= ringCount) return false;
  const uint16_t cap = RING_BUFFER_CAPACITY;
  uint16_t slot = (uint16_t)((ringTail + cap - 1 - i) % cap);
  memcpy(dst, ringSlotPtr(slot), SBD_MSG_SIZE);
  return true;
}

// Pop n newest messages (remove from tail). Safe if n > size (clamps to clear).
void ringBufferPopNewest(uint16_t n) {  // NEW
  if (n >= ringCount) {
    ringBufferClear();
    return;
  }
  const uint16_t cap = RING_BUFFER_CAPACITY;
  ringTail = (uint16_t)((ringTail + cap - (n % cap)) % cap);  // move tail backward
  ringCount -= n;
}

// ----------------------------------------------------------------------------
// Ring buffer introspection & pretty-print helpers (diagnostics)
// ----------------------------------------------------------------------------
void printRingBufferStatus(const char* where) {
  const uint16_t size = ringBufferSize();
  const uint16_t willSend = (size > SBD_MAX_MSGS) ? SBD_MAX_MSGS : size;

  DEBUG_PRINT("[Iridium] Info: ");
  DEBUG_PRINT(where);
  DEBUG_PRINT(" | ring size: ");
  DEBUG_PRINT(size);
  DEBUG_PRINT("/");
  DEBUG_PRINT(ringBufferCapacity());
  DEBUG_PRINT(" | head: ");
  DEBUG_PRINT(ringHead);
  DEBUG_PRINT(" | tail: ");
  DEBUG_PRINT(ringTail);
  DEBUG_PRINT(" | next TX: ");
  DEBUG_PRINT(willSend);
  DEBUG_PRINT(" msg (");
  DEBUG_PRINT(willSend * SBD_MSG_SIZE);
  DEBUG_PRINT("/");
  DEBUG_PRINT(SBD_BUF_BYTES);
  DEBUG_PRINT(" B) | dropped: ");
  DEBUG_PRINT(ringBufferDropped());
  DEBUG_PRINT(" | policy: ");
  DEBUG_PRINT((sendPolicy == SendPolicy::NEWEST_FIRST) ? "NEWEST_FIRST" : "OLDEST_FIRST");
  DEBUG_PRINTLN(".");
}

// Compact occupancy map
// Legend: H = head (oldest), T = tail (write position), ■ = occupied, · = empty
// Show-all by default; pass a smaller value to crop the view
void printRingBufferMap(int16_t maxSlots = -1) {
  const uint16_t cap = ringBufferCapacity();
  const uint16_t size = ringBufferSize();
  const uint16_t show = (maxSlots < 0) ? cap : (uint16_t)min<uint16_t>(cap, maxSlots);

  DEBUG_PRINT("[Iridium] Map: ");
  for (uint16_t i = 0; i < show; ++i) {
    const uint16_t slot = (ringHead + i) % cap;  // Rotated view keeps H visible
    const bool occ = (i < size);
    const bool isHead = (i == 0);
    const bool isTail = (slot == ringTail);

    if (isHead && isTail) DEBUG_PRINT("HT");
    else if (isHead) DEBUG_PRINT("H");
    else if (isTail) DEBUG_PRINT("T");
    else DEBUG_PRINT(occ ? "■" : "·");
  }

  if (show < cap) {
    const uint16_t dist = (ringTail + cap - ringHead) % cap;
    if (dist >= show) DEBUG_PRINT(" (T out of view)");
  }
  DEBUG_PRINTLN();
}

// ----------------------------------------------------------------------------
// Helpers to read little-endian fields from a 34-byte record (no structs)
// ----------------------------------------------------------------------------
static inline uint16_t rd_u16(const uint8_t* p) {
  uint16_t v;
  memcpy(&v, p, 2);
  return v;
}
static inline int16_t rd_i16(const uint8_t* p) {
  int16_t v;
  memcpy(&v, p, 2);
  return v;
}
static inline uint32_t rd_u32(const uint8_t* p) {
  uint32_t v;
  memcpy(&v, p, 4);
  return v;
}
static inline int32_t rd_i32(const uint8_t* p) {
  int32_t v;
  memcpy(&v, p, 4);
  return v;
}

// Offsets inside a 34-byte MO record
enum {
  OFF_UNIX = 0,
  OFF_TINT = 4,
  OFF_HUM = 6,
  OFF_PRES = 8,
  OFF_PITCH = 10,
  OFF_ROLL = 12,
  OFF_HEAD = 14,
  OFF_LAT = 16,
  OFF_LON = 20,
  OFF_SATS = 24,
  OFF_HDOP = 25,
  OFF_VBAT = 27,
  OFF_TXDUR = 29,
  OFF_TXRC = 31,
  OFF_ITER = 32
};

// ----------------------------------------------------------------------------
//  Human-readable record summaries
// ----------------------------------------------------------------------------
static inline void decodeMoRecord(const uint8_t* src, uint8_t* out) {
  memcpy(out, src, SBD_MSG_SIZE);
}

void printMoQueueSummary(const char* where) {
  const uint16_t size = ringBufferSize();
  DEBUG_PRINT("[Iridium] Info: ");
  DEBUG_PRINT(where);
  DEBUG_PRINT(" | queued: ");
  DEBUG_PRINT(size);
  DEBUG_PRINT(" msg (");
  DEBUG_PRINT((size_t)size * (size_t)SBD_MSG_SIZE);
  DEBUG_PRINT("/");
  DEBUG_PRINT(SBD_BUF_BYTES);
  DEBUG_PRINTLN(" B).");
}

void printMoRecordSummary(uint16_t logicalIndex, const uint8_t* raw) {
  uint8_t rec[SBD_MSG_SIZE];
  memcpy(rec, raw, SBD_MSG_SIZE);

  const uint32_t unixTime = rd_u32(&rec[OFF_UNIX]);
  const int16_t tIntRaw = rd_i16(&rec[OFF_TINT]);    // °C * 100
  const uint16_t hIntRaw = rd_u16(&rec[OFF_HUM]);    // % * 100
  const uint16_t pIntRaw = rd_u16(&rec[OFF_PRES]);   // (hPa - 850) * 100
  const int16_t pitchRaw = rd_i16(&rec[OFF_PITCH]);  // deg * 100
  const int16_t rollRaw = rd_i16(&rec[OFF_ROLL]);    // deg * 100
  const uint16_t headDeg = rd_u16(&rec[OFF_HEAD]);   // deg
  const int32_t latRaw = rd_i32(&rec[OFF_LAT]);      // deg * 1e6
  const int32_t lonRaw = rd_i32(&rec[OFF_LON]);      // deg * 1e6
  const uint8_t sats = rec[OFF_SATS];                // count
  const uint16_t hdopRaw = rd_u16(&rec[OFF_HDOP]);   // * 100
  const uint16_t vbatRaw = rd_u16(&rec[OFF_VBAT]);   // V * 100
  const uint16_t txDur = rd_u16(&rec[OFF_TXDUR]);    // s
  const uint8_t txRC = rec[OFF_TXRC];                // return code
  const uint16_t iter = rd_u16(&rec[OFF_ITER]);      // counter

  const float lat = (float)latRaw / 1000000.0f;
  const float lon = (float)lonRaw / 1000000.0f;
  const float vbat = (float)vbatRaw / 100.0f;
  const float hdop = (float)hdopRaw / 100.0f;
  const float tInt = (float)tIntRaw / 100.0f;
  const float hInt = (float)hIntRaw / 100.0f;
  const float pInt = 850.0f + ((float)pIntRaw / 100.0f);
  const float pitch = (float)pitchRaw / 100.0f;
  const float roll = (float)rollRaw / 100.0f;

  DEBUG_PRINT("[Queue] #");
  DEBUG_PRINT(logicalIndex);
  DEBUG_PRINT(" | unix: ");
  DEBUG_PRINT(unixTime);

  DEBUG_PRINT(" | lat: ");
  DEBUG_PRINT_DEC(lat, 6);
  DEBUG_PRINT(" | lon: ");
  DEBUG_PRINT_DEC(lon, 6);

  DEBUG_PRINT(" | Vbat: ");
  DEBUG_PRINT_DEC(vbat, 2);
  DEBUG_PRINT(" V");

  DEBUG_PRINT(" | sats: ");
  DEBUG_PRINT(sats);
  DEBUG_PRINT(" | hdop: ");
  DEBUG_PRINT_DEC(hdop, 2);

  DEBUG_PRINT(" | T: ");
  DEBUG_PRINT_DEC(tInt, 2);
  DEBUG_PRINT("°C");
  DEBUG_PRINT(" | RH: ");
  DEBUG_PRINT_DEC(hInt, 2);
  DEBUG_PRINT("%");
  DEBUG_PRINT(" | P: ");
  DEBUG_PRINT_DEC(pInt, 2);
  DEBUG_PRINT(" hPa");

  DEBUG_PRINT(" | pitch: ");
  DEBUG_PRINT_DEC(pitch, 2);
  DEBUG_PRINT("°");
  DEBUG_PRINT(" | roll: ");
  DEBUG_PRINT_DEC(roll, 2);
  DEBUG_PRINT("°");
  DEBUG_PRINT(" | head: ");
  DEBUG_PRINT(headDeg);
  DEBUG_PRINT(" °");

  DEBUG_PRINT(" | txDur: ");
  DEBUG_PRINT(txDur);
  DEBUG_PRINT(" s");
  DEBUG_PRINT(" | txRC: ");
  DEBUG_PRINT(txRC);
  DEBUG_PRINT(" | iter: ");
  DEBUG_PRINT(iter);

  DEBUG_PRINTLN();
}

// Preview first N (oldest) records
void printRingBufferPreview(uint16_t n) {
  const uint16_t size = ringBufferSize();
  if (size == 0) {
    DEBUG_PRINTLN("[Iridium] Info: Queue preview: empty.");
    return;
  }
  const uint16_t take = (n > size) ? size : n;
  DEBUG_PRINT("[Iridium] Info: Previewing ");
  DEBUG_PRINT(take);
  DEBUG_PRINT(" oldest record(s) out of ");
  DEBUG_PRINT(size);
  DEBUG_PRINTLN("...");

  for (uint16_t i = 0; i < take; ++i) {
    uint8_t tmp[SBD_MSG_SIZE];
    ringBufferPeek(i, tmp);
    printMoRecordSummary(i, tmp);
  }
}

// Preview next TX content for oldest-first policy
void printNextTransmitPreview() {
  const uint16_t size = ringBufferSize();
  if (size == 0) {
    DEBUG_PRINTLN("[Iridium] Info: Next TX preview: empty.");
    return;
  }
  const uint16_t take = (size > SBD_MAX_MSGS) ? SBD_MAX_MSGS : size;
  DEBUG_PRINT("[Iridium] Info: Next TX will include ");
  DEBUG_PRINT(take);
  DEBUG_PRINT(" record(s). Details:");
  DEBUG_PRINTLN();

  for (uint16_t i = 0; i < take; ++i) {
    uint8_t tmp[SBD_MSG_SIZE];
    ringBufferPeek(i, tmp);
    printMoRecordSummary(i, tmp);
  }
}

// ----------------------------------------------------------------------------
// Preview next TX content for newest-first policy
// Packs oldest->newest inside the selected newest block
// ----------------------------------------------------------------------------
void printNextTransmitPreviewNewest() {
  const uint16_t size = ringBufferSize();
  if (size == 0) {
    DEBUG_PRINTLN("[Iridium] Info: Next TX preview: empty.");
    return;
  }
  const uint16_t take = (size > SBD_MAX_MSGS) ? SBD_MAX_MSGS : size;
  DEBUG_PRINT("[Iridium] Info: Next TX (newest policy) will include ");
  DEBUG_PRINT(take);
  DEBUG_PRINT(" record(s). Details:");
  DEBUG_PRINTLN();

  for (uint16_t j = 0; j < take; ++j) {
    uint16_t i_from_newest = (uint16_t)((take - 1) - j);  // Oldest within newest block first
    uint8_t tmp[SBD_MSG_SIZE];
    ringBufferPeekNewest(i_from_newest, tmp);
    printMoRecordSummary(j, tmp);
  }
}

// Dump the full 340-byte MO window (prints all bytes)
void printMoWindowHex(const uint8_t* buf, size_t len /*=SBD_BUF_BYTES*/) {
  DEBUG_PRINTLN("--------------------------------------------------------------------------------");
  DEBUG_PRINTLN("MO-SBD Transmit buffer (full window)");
  DEBUG_PRINTLN("--------------------------------------------------------------------------------");
  DEBUG_PRINTLN("Byte\tHex");
  for (size_t i = 0; i < len; ++i) {
    DEBUG_PRINT(i);
    DEBUG_PRINT("\t0x");
    DEBUG_PRINTLN_HEX(buf[i]);
  }
}

// ----------------------------------------------------------------------------
// Iridium configuration
// ----------------------------------------------------------------------------
void configureIridium() {
  modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE);  // Use default battery power profile
  modem.adjustSendReceiveTimeout(iridiumTimeout);            // Set Iridium TX/RX timeout
  modem.adjustStartupTimeout(IridiumStartup);                // Set startup timeout

  // Sanity log
  DEBUG_PRINT("[Iridium] Info: MO msg size = ");
  DEBUG_PRINT(SBD_MSG_SIZE);
  DEBUG_PRINT(" B, MO window = ");
  DEBUG_PRINT(SBD_BUF_BYTES);
  DEBUG_PRINT(" B, max msgs per TX = ");
  DEBUG_PRINT(SBD_MAX_MSGS);
  DEBUG_PRINT(", ring capacity = ");
  DEBUG_PRINT(ringBufferCapacity());
  DEBUG_PRINTLN(" msgs.");

  DEBUG_PRINTLN("[Iridium] Info: RockBLOCK 9603 initialized.");
}

// ----------------------------------------------------------------------------
// Build a MO payload from the oldest queued messages
// ----------------------------------------------------------------------------
uint16_t buildMoPayloadFromQueue(uint8_t* dst, size_t dstBytes, size_t* outBytes) {
  const uint16_t queued = ringBufferSize();
  if (queued == 0) {
    *outBytes = 0;
    return 0;
  }

  uint16_t take = (queued > SBD_MAX_MSGS) ? SBD_MAX_MSGS : queued;
  size_t needBytes = (size_t)take * (size_t)SBD_MSG_SIZE;

  if (needBytes > dstBytes) {
    uint16_t safeTake = (uint16_t)(dstBytes / SBD_MSG_SIZE);
    take = safeTake;
    needBytes = (size_t)take * (size_t)SBD_MSG_SIZE;
  }

  // Pack oldest-first into dst
  for (uint16_t i = 0; i < take; ++i) {
    ringBufferPeek(i, dst + (i * SBD_MSG_SIZE));
  }

  *outBytes = needBytes;
  return take;
}

// ----------------------------------------------------------------------------
// Build a MO payload from the newest queued messages
// Packs oldest->newest inside the selected newest block
// ----------------------------------------------------------------------------
uint16_t buildMoPayloadFromNewest(uint8_t* dst, size_t dstBytes, size_t* outBytes) {  // NEW
  const uint16_t queued = ringBufferSize();
  if (queued == 0) {
    *outBytes = 0;
    return 0;
  }

  uint16_t take = (queued > SBD_MAX_MSGS) ? SBD_MAX_MSGS : queued;
  size_t needBytes = (size_t)take * (size_t)SBD_MSG_SIZE;

  if (needBytes > dstBytes) {
    uint16_t safeTake = (uint16_t)(dstBytes / SBD_MSG_SIZE);
    take = safeTake;
    needBytes = (size_t)take * (size_t)SBD_MSG_SIZE;
  }

  // Select newest take records and pack them in chronological order within the batch
  for (uint16_t j = 0; j < take; ++j) {
    uint16_t i_from_newest = (uint16_t)((take - 1) - j);  // oldest within newest block first
    ringBufferPeekNewest(i_from_newest, dst + (j * SBD_MSG_SIZE));
  }

  *outBytes = needBytes;
  return take;
}

// ----------------------------------------------------------------------------
// Write one message into the queue (called each sampling interval)
// ----------------------------------------------------------------------------
void writeBuffer() {
  iterationCounter++;
  transmitCounter++;

  DEBUG_PRINT("[Iridium] Info: Transmit counter ");
  DEBUG_PRINT(transmitCounter);
  DEBUG_PRINT(" of ");
  DEBUG_PRINT(transmitInterval);
  DEBUG_PRINTLN(".");

  DEBUG_PRINTLN("[Iridium] Info: Writing new MO-SBD message to queue...");

  // Populate dynamic fields (global union lives elsewhere; safe to use here)
  moSbdMessage.voltage = readBattery() * 100;
  moSbdMessage.iterationCounter = iterationCounter;

  // Enqueue serialized record
  ringBufferPush(moSbdMessage.bytes);

  // Optional: print the record
  printMoSbd();

  // Clear message struct after copying
  moSbdMessage = {};

  // Queue summaries after write
  printMoQueueSummary("writeBuffer");
  printRingBufferStatus("writeBuffer");
  printRingBufferMap();  // Visual occupancy map (optional)
  // printRingBufferPreview(3);
}

// ----------------------------------------------------------------------------
// Transmit messages according to selected policy
// ----------------------------------------------------------------------------
void transmitData() {
  if (ringBufferEmpty()) {
    DEBUG_PRINTLN("[Iridium] Info: No queued messages to send.");
    transmitCounter = 0;
    return;
  }

  // Stage payload (up to 10 messages = 340 bytes)
  size_t moBytes = 0;
  memset(moSbdBuffer, 0x00, sizeof(moSbdBuffer));

  uint16_t stagedMsgs = 0;
  if (sendPolicy == SendPolicy::NEWEST_FIRST) {
    stagedMsgs = buildMoPayloadFromNewest(moSbdBuffer, sizeof(moSbdBuffer), &moBytes);
  } else {
    stagedMsgs = buildMoPayloadFromQueue(moSbdBuffer, sizeof(moSbdBuffer), &moBytes);
  }

  DEBUG_PRINT("[Iridium] Info: Staging ");
  DEBUG_PRINT(stagedMsgs);
  DEBUG_PRINT(" messages (");
  DEBUG_PRINT(moBytes);
  DEBUG_PRINTLN(" bytes) for TX.");

  printRingBufferStatus("transmitData(pre)");
  if (sendPolicy == SendPolicy::NEWEST_FIRST) {
    printNextTransmitPreviewNewest();
  } else {
    printNextTransmitPreview();
  }
  // printMoWindowHex(moSbdBuffer, SBD_BUF_BYTES);

  // Power up modem and open port
  unsigned long startTime = millis();
  enable5V();
  IRIDIUM_PORT.begin(19200);
  pinPeripheral(PIN_IRIDIUM_TX, PIO_SERCOM);
  pinPeripheral(PIN_IRIDIUM_RX, PIO_SERCOM);

  DEBUG_PRINTLN("[Iridium] Info: Initializing Iridium modem...");
  int rc = modem.begin();

  if (rc != ISBD_SUCCESS) {
    DEBUG_PRINTLN("[Iridium] Error: Failed to initialize Iridium modem.");
    printIridiumError(rc);
    transmitCounter = 0;
  } else {
    memset(mtSbdBuffer, 0x00, sizeof(mtSbdBuffer));
    mtSbdBufferSize = sizeof(mtSbdBuffer);

    DEBUG_PRINTLN("[Iridium] Info: Attempting to transmit message...");
    rc = modem.sendReceiveSBDBinary(moSbdBuffer, moBytes, mtSbdBuffer, mtSbdBufferSize);

    if (rc == ISBD_SUCCESS) {
      DEBUG_PRINTLN("[Iridium] Info: MO-SBD message transmission successful!");
      blinkLed(10, 250);

      // Remove only what we sent from the correct end
      if (sendPolicy == SendPolicy::NEWEST_FIRST) {
        ringBufferPopNewest(stagedMsgs);
      } else {
        ringBufferPop(stagedMsgs);
      }

      transmitCounter = 0;  // start a fresh window

      // Process MT
      if (mtSbdBufferSize == 7) {
        const uint8_t* b = mtSbdBuffer;
        bool ok =
          (b[0] <= 2) && (b[1] <= 31)
          && (b[2] < 24) && (b[3] < 60)
          && (b[4] >= 1 && b[4] <= 10)
          && (b[5] <= 10) && (b[6] == 0 || b[6] == 255);

        DEBUG_PRINTLN("[Iridium] Info: MT-SBD bytes received (7).");
        printMtSbdBuffer();

        if (ok) {
          alarmMode = b[0];
          alarmIntervalDay = b[1];
          alarmIntervalHour = b[2];
          alarmIntervalMinute = b[3];
          transmitInterval = b[4];
          transmitReattempts = b[5];
          resetFlag = b[6];

          printMtSbd();
          DEBUG_PRINTLN("[Iridium] Info: System parameters updated from MT-SBD message.");
        } else {
          DEBUG_PRINTLN("[Iridium] Warning: Received values exceed accepted range!");
        }
      }
    } else {
      DEBUG_PRINTLN("[Iridium] Warning: Transmission failed.");
      printIridiumError(rc);
      blinkLed(5, 1000);
      transmitCounter = 0;  // Wait a full interval before next attempt
    }
  }

  // Record return code in the message struct (for telemetry)
  moSbdMessage.transmitStatus = rc;

  // Sleep modem
  DEBUG_PRINTLN("[Iridium] Info: Putting modem to sleep...");
  rc = modem.sleep();
  if (rc != ISBD_SUCCESS) {
    DEBUG_PRINTLN("[Iridium] Error: Could not put modem to sleep.");
    printIridiumError(rc);
  } else {
    DEBUG_PRINTLN("[Iridium] Info: Modem put to sleep successfully.");
  }

  // Close serial and power down
  IRIDIUM_PORT.end();
  disable5V();

  // Execution time
  timer.iridium = millis() - startTime;
  moSbdMessage.transmitDuration = timer.iridium / 1000;

  // Print current settings
  printSettings();

  // Forced reset request (from MT)
  if (resetFlag) {
    DEBUG_PRINTLN("[Iridium] Info: Forced system reset...");
    digitalWrite(LED_BUILTIN, HIGH);
    while (true) { /* wait for WDT reset */
    }
  }
}

// ----------------------------------------------------------------------------
// Callbacks and error helpers
// ----------------------------------------------------------------------------
bool ISBDCallback() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > 1000) {
    previousMillis = currentMillis;
    resetWdt();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
  return true;
}

#if DEBUG_IRIDIUM
void ISBDConsoleCallback(IridiumSBD* /*device*/, char c) {
  DEBUG_WRITE(c);
}
void ISBDDiagsCallback(IridiumSBD* /*device*/, char c) {
  DEBUG_WRITE(c);
}
#endif

void IridiumSBD::setSleepPin(uint8_t enable) {
  if (enable == HIGH) {
    digitalWrite(this->sleepPin, LOW);
    diagprint(F("Modem is awake.\r\n"));
  } else {
    digitalWrite(this->sleepPin, HIGH);
    diagprint(F("Modem is asleep.\r\n"));
  }
}

void printIridiumError(int code) {
  DEBUG_PRINT("[Iridium] Error ");
  DEBUG_PRINT(code);
  DEBUG_PRINT(": ");
  DEBUG_PRINTLN(iridiumErrorDescription(code));
}

const char* iridiumErrorDescription(int code) {
  switch (code) {
    case ISBD_SUCCESS: return "Success.";
    case ISBD_ALREADY_AWAKE: return "Modem already awake.";
    case ISBD_SERIAL_FAILURE: return "Serial communication failure.";
    case ISBD_PROTOCOL_ERROR: return "AT command protocol error.";
    case ISBD_CANCELLED: return "Operation cancelled.";
    case ISBD_NO_MODEM_DETECTED: return "No modem detected. Check wiring.";
    case ISBD_SBDIX_FATAL_ERROR: return "Fatal error during SBDIX.";
    case ISBD_SENDRECEIVE_TIMEOUT: return "Send/receive timed out.";
    case ISBD_RX_OVERFLOW: return "Receive buffer overflow.";
    case ISBD_REENTRANT: return "Reentrant call detected.";
    case ISBD_IS_ASLEEP: return "Modem is asleep.";
    case ISBD_NO_SLEEP_PIN: return "Sleep pin not configured.";
    case ISBD_NO_NETWORK: return "No network available.";
    case ISBD_MSG_TOO_LONG: return "Message too long.";
    default: return "Unknown error.";
  }
}
