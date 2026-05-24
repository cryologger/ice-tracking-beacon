/*
  Iridium Module

  This module configures and manages the RockBLOCK 9603 Iridium modem.
  It provides functions to set up the modem, build and transmit SBD messages,
  and handle incoming MT messages. Callbacks for the IridiumSBD library are 
  included for debugging and non-blocking transmission or GNSS acquisition.

  Ring Buffer Design:
  - Store messages in a RAM ring buffer (newest-first).
  - Each sample enqueues one SBD record (SBD_MO_SIZE bytes).
  - On each TX, send up to SBD_MAX_MSGS newest messages, packed
    chronologically (oldest-within-batch first) into the MO window.
  - On success, remove exactly the number sent from the tail.
  - On failure, keep all queued messages.
*/

// ----------------------------------------------------------------------------
// Ring buffer storage and indices
// ----------------------------------------------------------------------------
#ifndef RING_BUFFER_CAPACITY
// Memory usage = RING_BUFFER_CAPACITY * SBD_MO_SIZE bytes
#define RING_BUFFER_CAPACITY 100
#endif

static uint8_t ringStorage[RING_BUFFER_CAPACITY * SBD_MO_SIZE];

static uint16_t ringHead = 0;     // Index of oldest message (0..capacity-1)
static uint16_t ringTail = 0;     // Index to write the next message
static uint16_t ringCount = 0;    // Number of valid messages in the ring
static uint32_t ringDropped = 0;  // Oldest records overwritten when full

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
  // ringDropped is intentionally preserved — it is useful long-term telemetry.
}

// Returns a pointer to the start of slot i in flat storage.
static uint8_t* ringSlotPtr(uint16_t slotIndex) {
  return &ringStorage[(size_t)slotIndex * (size_t)SBD_MO_SIZE];
}

// Push one message (SBD_MO_SIZE bytes) at the tail.
// When full, overwrite the oldest message (advance head) and count the drop.
void ringBufferPush(const uint8_t* bytes) {
  if (ringBufferFull()) {
    ringHead = (uint16_t)((ringHead + 1) % RING_BUFFER_CAPACITY);
    ringDropped++;
    // ringCount stays at capacity.
  } else {
    ringCount++;
  }
  memcpy(ringSlotPtr(ringTail), bytes, SBD_MO_SIZE);
  ringTail = (uint16_t)((ringTail + 1) % RING_BUFFER_CAPACITY);
}

// Peek at message at offset i from the oldest (0 = oldest). Returns false if out of range.
bool ringBufferPeek(uint16_t i, uint8_t* dst) {
  if (i >= ringCount) return false;
  uint16_t slot = (uint16_t)((ringHead + i) % RING_BUFFER_CAPACITY);
  memcpy(dst, ringSlotPtr(slot), SBD_MO_SIZE);
  return true;
}

// Peek at message at offset i from the newest (0 = newest). Returns false if out of range.
bool ringBufferPeekNewest(uint16_t i, uint8_t* dst) {
  if (i >= ringCount) return false;
  const uint16_t cap = RING_BUFFER_CAPACITY;
  uint16_t slot = (uint16_t)((ringTail + cap - 1 - i) % cap);
  memcpy(dst, ringSlotPtr(slot), SBD_MO_SIZE);
  return true;
}

// Pop n newest messages from the tail. Clamps safely if n >= size.
void ringBufferPopNewest(uint16_t n) {
  if (n >= ringCount) {
    ringBufferClear();
    return;
  }
  const uint16_t cap = RING_BUFFER_CAPACITY;
  ringTail = (uint16_t)((ringTail + cap - (n % cap)) % cap);
  ringCount -= n;
}

// ----------------------------------------------------------------------------
// Ring buffer diagnostics
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
  DEBUG_PRINT(willSend * SBD_MO_SIZE);
  DEBUG_PRINT("/");
  DEBUG_PRINT(SBD_MO_BUF_BYTES);
  DEBUG_PRINT(" B) | dropped: ");
  DEBUG_PRINTLN(ringBufferDropped());
}

// Compact occupancy map.
// Legend: H = head (oldest), T = tail (write position), # = occupied, . = empty
// Pass maxSlots >= 0 to crop the view; -1 (default) shows all slots.
void printRingBufferMap(int16_t maxSlots = -1) {
  const uint16_t cap = ringBufferCapacity();
  const uint16_t size = ringBufferSize();
  const uint16_t show = (maxSlots < 0) ? cap : (uint16_t)min<uint16_t>(cap, maxSlots);

  DEBUG_PRINT("[Iridium] Map: ");
  for (uint16_t i = 0; i < show; ++i) {
    const uint16_t slot = (ringHead + i) % cap;
    const bool occ = (i < size);
    const bool isHead = (i == 0);
    const bool isTail = (slot == ringTail);

    if (isHead && isTail) DEBUG_PRINT("HT");
    else if (isHead) DEBUG_PRINT("H");
    else if (isTail) DEBUG_PRINT("T");
    else DEBUG_PRINT(occ ? "#" : ".");
  }

  if (show < cap) {
    const uint16_t dist = (ringTail + cap - ringHead) % cap;
    if (dist >= show) DEBUG_PRINT(" (T out of view)");
  }
  DEBUG_PRINTLN();
}

// ----------------------------------------------------------------------------
// Helpers to read little-endian fields from a packed 34-byte record.
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

// Byte offsets inside a 34-byte MO record (mirrors SBD_MO_MESSAGE layout).
enum {
  OFF_UNIX = 0,    // uint32 — Unix epoch
  OFF_TINT = 4,    // int16  — temperature * 100 (°C)
  OFF_HUM = 6,     // uint16 — humidity * 100 (%)
  OFF_PRES = 8,    // uint16 — (pressure - 850) * 100 (hPa)
  OFF_PITCH = 10,  // int16  — pitch * 100 (°)
  OFF_ROLL = 12,   // int16  — roll * 100 (°)
  OFF_HEAD = 14,   // uint16 — heading (°)
  OFF_LAT = 16,    // int32  — latitude * 1e6 (°)
  OFF_LON = 20,    // int32  — longitude * 1e6 (°)
  OFF_SATS = 24,   // uint8  — satellite count
  OFF_HDOP = 25,   // uint16 — HDOP * 100
  OFF_VBAT = 27,   // uint16 — battery voltage * 100 (V)
  OFF_TXDUR = 29,  // uint16 — last transmit duration (s)
  OFF_TXRC = 31,   // uint8  — last transmit return code
  OFF_ITER = 32    // uint16 — iteration counter
};

// ----------------------------------------------------------------------------
// Human-readable record summary (used by queue preview).
// ----------------------------------------------------------------------------
void printMoRecordSummary(uint16_t logicalIndex, const uint8_t* raw) {
  uint8_t rec[SBD_MO_SIZE];
  memcpy(rec, raw, SBD_MO_SIZE);

  const float lat = (float)rd_i32(&rec[OFF_LAT]) / 1000000.0f;
  const float lon = (float)rd_i32(&rec[OFF_LON]) / 1000000.0f;
  const float vbat = (float)rd_u16(&rec[OFF_VBAT]) / 100.0f;
  const float hdop = (float)rd_u16(&rec[OFF_HDOP]) / 100.0f;
  const float tInt = (float)rd_i16(&rec[OFF_TINT]) / 100.0f;
  const float hInt = (float)rd_u16(&rec[OFF_HUM]) / 100.0f;
  const float pInt = 850.0f + (float)rd_u16(&rec[OFF_PRES]) / 100.0f;
  const float pitch = (float)rd_i16(&rec[OFF_PITCH]) / 100.0f;
  const float roll = (float)rd_i16(&rec[OFF_ROLL]) / 100.0f;

  DEBUG_PRINT("[Queue] #");
  DEBUG_PRINT(logicalIndex);
  DEBUG_PRINT(" | unix: ");
  DEBUG_PRINT(rd_u32(&rec[OFF_UNIX]));
  DEBUG_PRINT(" | lat: ");
  DEBUG_PRINT_DEC(lat, 6);
  DEBUG_PRINT(" | lon: ");
  DEBUG_PRINT_DEC(lon, 6);
  DEBUG_PRINT(" | Vbat: ");
  DEBUG_PRINT_DEC(vbat, 2);
  DEBUG_PRINT(" V");
  DEBUG_PRINT(" | sats: ");
  DEBUG_PRINT(rec[OFF_SATS]);
  DEBUG_PRINT(" | hdop: ");
  DEBUG_PRINT_DEC(hdop, 2);
  DEBUG_PRINT(" | T: ");
  DEBUG_PRINT_DEC(tInt, 2);
  DEBUG_PRINT(" C");
  DEBUG_PRINT(" | RH: ");
  DEBUG_PRINT_DEC(hInt, 2);
  DEBUG_PRINT("%");
  DEBUG_PRINT(" | P: ");
  DEBUG_PRINT_DEC(pInt, 2);
  DEBUG_PRINT(" hPa");
  DEBUG_PRINT(" | pitch: ");
  DEBUG_PRINT_DEC(pitch, 2);
  DEBUG_PRINT(" deg");
  DEBUG_PRINT(" | roll: ");
  DEBUG_PRINT_DEC(roll, 2);
  DEBUG_PRINT(" deg");
  DEBUG_PRINT(" | head: ");
  DEBUG_PRINT(rd_u16(&rec[OFF_HEAD]));
  DEBUG_PRINT(" deg");
  DEBUG_PRINT(" | txDur: ");
  DEBUG_PRINT(rd_u16(&rec[OFF_TXDUR]));
  DEBUG_PRINT(" s");
  DEBUG_PRINT(" | txRC: ");
  DEBUG_PRINT(rec[OFF_TXRC]);
  DEBUG_PRINT(" | iter: ");
  DEBUG_PRINT(rd_u16(&rec[OFF_ITER]));
  DEBUG_PRINTLN();
}

// Preview the next N records that will be transmitted (newest N, packed chronologically).
void printNextTransmitPreview() {
  const uint16_t size = ringBufferSize();
  if (size == 0) {
    DEBUG_PRINTLN("[Iridium] Info: Next TX preview: empty.");
    return;
  }
  const uint16_t take = (size > SBD_MAX_MSGS) ? SBD_MAX_MSGS : size;
  DEBUG_PRINT("[Iridium] Info: Next TX will include ");
  DEBUG_PRINT(take);
  DEBUG_PRINTLN(" record(s):");

  for (uint16_t j = 0; j < take; ++j) {
    uint8_t tmp[SBD_MO_SIZE];
    // Walk newest-to-oldest, but display oldest-within-batch first.
    ringBufferPeekNewest((uint16_t)((take - 1) - j), tmp);
    printMoRecordSummary(j, tmp);
  }
}

// Dump the full MO window as a hex table (call-site opt-in).
void printMoWindowHex(const uint8_t* buf, size_t len) {
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
  modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE);
  modem.adjustSendReceiveTimeout(iridiumTimeout);
  modem.adjustStartupTimeout(iridiumStartup);

  DEBUG_PRINT("[Iridium] Info: MO msg size = ");
  DEBUG_PRINT(SBD_MO_SIZE);
  DEBUG_PRINT(" B, MO window = ");
  DEBUG_PRINT(SBD_MO_BUF_BYTES);
  DEBUG_PRINT(" B, max msgs per TX = ");
  DEBUG_PRINT(SBD_MAX_MSGS);
  DEBUG_PRINT(", ring capacity = ");
  DEBUG_PRINT(ringBufferCapacity());
  DEBUG_PRINTLN(" msgs.");
  DEBUG_PRINTLN("[Iridium] Info: RockBLOCK 9603 initialized.");
}

// ----------------------------------------------------------------------------
// Build the MO payload from the newest queued messages.
// Selects up to SBD_MAX_MSGS newest records and packs them chronologically
// (oldest-within-batch first) into dst. Returns the number of messages staged.
// ----------------------------------------------------------------------------
uint16_t buildMoPayload(uint8_t* dst, size_t dstBytes, size_t* outBytes) {
  const uint16_t queued = ringBufferSize();
  if (queued == 0) {
    *outBytes = 0;
    return 0;
  }

  uint16_t take = (queued > SBD_MAX_MSGS) ? SBD_MAX_MSGS : queued;
  size_t needBytes = (size_t)take * (size_t)SBD_MO_SIZE;

  // Safety clamp: ensure payload fits in the destination buffer.
  if (needBytes > dstBytes) {
    take = (uint16_t)(dstBytes / SBD_MO_SIZE);
    needBytes = (size_t)take * (size_t)SBD_MO_SIZE;
  }

  // Pack oldest-within-batch first so the receiver sees chronological order.
  for (uint16_t j = 0; j < take; ++j) {
    ringBufferPeekNewest((uint16_t)((take - 1) - j), dst + (j * SBD_MO_SIZE));
  }

  *outBytes = needBytes;
  return take;
}

// ----------------------------------------------------------------------------
// Enqueue one message (called each sampling interval).
// ----------------------------------------------------------------------------
void writeBuffer() {
  iterationCounter++;
  transmitCounter++;

  DEBUG_PRINT("[Iridium] Info: Transmit counter ");
  DEBUG_PRINT(transmitCounter);
  DEBUG_PRINT(" of ");
  DEBUG_PRINTLN(transmitInterval);

  // Populate fields that are only known at queue-time
  moSbdMessage.voltage = (uint16_t)lroundf(readBattery() * 100.0f);
  moSbdMessage.iterationCounter = iterationCounter;

  ringBufferPush(moSbdMessage.bytes);
  printMoSbd();

  // Clear struct so transmitStatus/transmitDuration from the previous TX cycle
  // do not persist beyond this point (they carry forward intentionally for one
  // cycle, then are zeroed here)
  moSbdMessage = {};


#if DEBUG_IRIDIUM
  printRingBufferStatus("writeBuffer");
  printRingBufferMap();
#endif
}

// ----------------------------------------------------------------------------
// Transmit queued messages over Iridium SBD.
// ----------------------------------------------------------------------------
void transmitData() {
  if (ringBufferEmpty()) {
    DEBUG_PRINTLN("[Iridium] Info: No queued messages to send.");
    transmitCounter = 0;
    return;
  }

  // Build the payload from the newest queued records
  size_t moBytes = 0;
  memset(moSbdBuffer, 0x00, sizeof(moSbdBuffer));
  uint16_t stagedMsgs = buildMoPayload(moSbdBuffer, sizeof(moSbdBuffer), &moBytes);

  printMoWindowHex(moSbdBuffer, moBytes);

  DEBUG_PRINT("[Iridium] Info: Staging ");
  DEBUG_PRINT(stagedMsgs);
  DEBUG_PRINT(" messages (");
  DEBUG_PRINT(moBytes);
  DEBUG_PRINTLN(" bytes) for TX.");

  printRingBufferStatus("transmitData(pre)");
  printNextTransmitPreview();

  // Power up modem and open serial port
  uint32_t startTime = millis();
  enable5V();
  myDelay(100);

  pinPeripheral(PIN_IRIDIUM_TX, PIO_SERCOM);
  pinPeripheral(PIN_IRIDIUM_RX, PIO_SERCOM);
  IRIDIUM_PORT.begin(19200);

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

      ringBufferPopNewest(stagedMsgs);
      transmitCounter = 0;

      // Process any inbound MT message
      if (mtSbdBufferSize == sizeof(SBD_MT_MESSAGE)) {
        memcpy(mtSbdMessage.bytes, mtSbdBuffer, sizeof(SBD_MT_MESSAGE));

        const uint8_t* b = mtSbdBuffer;

        bool alarmOk = false;
        switch (b[0]) {
          case MINUTE: alarmOk = (b[1] == 0) && (b[2] == 0) && (b[3] >= 1 && b[3] <= 59); break;
          case HOURLY: alarmOk = (b[1] == 0) && (b[2] >= 1 && b[2] <= 23) && (b[3] == 0); break;
          case DAILY: alarmOk = (b[1] == 1) && (b[2] == 0) && (b[3] == 0); break;
          default: alarmOk = false; break;
        }

        const bool ok = alarmOk
                        && (b[4] >= 1 && b[4] <= 10)
                        && (b[5] == 0 || b[5] == 255);

        DEBUG_PRINTLN("[Iridium] Info: MT-SBD message received.");
        printMtSbdBuffer();

        if (ok) {
          alarmMode = b[0];
          alarmIntervalDay = b[1];
          alarmIntervalHour = b[2];
          alarmIntervalMinute = b[3];
          transmitInterval = b[4];
          resetFlag = b[5];
          printMtSbd();
          DEBUG_PRINTLN("[Iridium] Info: System parameters updated from MT-SBD message.");
        } else {
          DEBUG_PRINTLN("[Iridium] Warning: MT-SBD values out of accepted range. Ignoring.");
        }
      }
    } else {
      DEBUG_PRINTLN("[Iridium] Warning: Transmission failed.");
      printIridiumError(rc);
      blinkLed(5, 1000);
      transmitCounter = 0;
    }
  }

  // Store TX outcome for inclusion in the next queued record (intentional one-cycle lag).
  moSbdMessage.transmitStatus = rc;

  // Put modem to sleep and power down.
  DEBUG_PRINTLN("[Iridium] Info: Putting modem to sleep...");
  rc = modem.sleep();
  if (rc != ISBD_SUCCESS) {
    DEBUG_PRINTLN("[Iridium] Error: Could not put modem to sleep.");
    printIridiumError(rc);
  } else {
    DEBUG_PRINTLN("[Iridium] Info: Modem put to sleep successfully.");
  }

  IRIDIUM_PORT.end();
  disable5V();

  timer.iridium = millis() - startTime;
  moSbdMessage.transmitDuration = timer.iridium / 1000;

  printSettings();

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
  uint32_t currentMillis = millis();
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

// ----------------------------------------------------------------------------
// RockBLOCK sleep control
// ----------------------------------------------------------------------------
void IridiumSBD::setSleepPin(uint8_t enable) {
#if ROCKBLOCK_VERSION_3F

  // RockBLOCK 9603N version 3.F uses inverted logic through TN0702 N-FET
  if (enable == HIGH) {
    digitalWrite(this->sleepPin, LOW);
    diagprint(F("Modem wake requested.\r\n"));
  } else {
    digitalWrite(this->sleepPin, HIGH);
    diagprint(F("Modem sleep requested.\r\n"));
  }

#else

  // Earlier RockBLOCK versions use normal logic
  digitalWrite(this->sleepPin, enable);

  if (enable == HIGH) {
    diagprint(F("Modem wake requested.\r\n"));
  } else {
    diagprint(F("Modem sleep requested.\r\n"));
  }

#endif
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