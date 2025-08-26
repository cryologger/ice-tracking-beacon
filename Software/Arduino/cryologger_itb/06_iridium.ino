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
  - On each transmit window, send up to SBD_MAX_MSGS (10) oldest messages.
  - On success, pop exactly the number sent. On failure, keep them.
*/

// ----------------------------------------------------------------------------
// Ring buffer for SBD messages (backing store for reliability)
// ----------------------------------------------------------------------------
#ifndef RING_BUFFER_CAPACITY
// Max messages to hold in RAM beyond a single 340-byte Iridium window.
// Tune as you like; memory usage = RING_BUFFER_CAPACITY * SBD_MSG_SIZE bytes.
#define RING_BUFFER_CAPACITY 100
#endif

// Flat storage: [capacity][SBD_MSG_SIZE]
static uint8_t ringStorage[RING_BUFFER_CAPACITY * SBD_MSG_SIZE];

// Indices and counters
static uint16_t ringHead = 0;    // index of oldest message (0..capacity-1)
static uint16_t ringTail = 0;    // index to write the next message
static uint16_t ringCount = 0;   // number of valid messages in the ring
static uint32_t ringDropped = 0; // how many oldest records we overwrote

// ----------------------------------------------------------------------------
// Ring buffer utilities
// ----------------------------------------------------------------------------
uint16_t ringBufferCapacity() { return (uint16_t)RING_BUFFER_CAPACITY; }
uint16_t ringBufferSize()     { return ringCount; }
bool     ringBufferEmpty()    { return ringCount == 0; }
bool     ringBufferFull()     { return ringCount >= RING_BUFFER_CAPACITY; }
uint32_t ringBufferDropped()  { return ringDropped; }

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
// Ring buffer introspection & pretty-print helpers (diagnostics)
// Prints a one-line status with capacity/size, head/tail positions, how many
// messages will be included in the next MO window, and how many were dropped.
// ----------------------------------------------------------------------------
void printRingBufferStatus(const char* where) {
  const uint16_t size     = ringBufferSize();
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
  DEBUG_PRINTLN(".");
}

// ----------------------------------------------------------------------------
// printRingBufferMap
//  Compact “map” of occupancy relative to head/tail for quick backlog view.
//  Legend:
//    H = head (oldest)
//    T = tail (write position)
//    ■ = occupied slot
//    · = empty slot
// ----------------------------------------------------------------------------
void printRingBufferMap(uint16_t maxSlots = 64) {
  const uint16_t cap  = ringBufferCapacity();
  const uint16_t size = ringBufferSize();

  const uint16_t show = (cap < maxSlots) ? cap : maxSlots;
  DEBUG_PRINT("[Iridium] Map: ");
  for (uint16_t i = 0; i < show; ++i) {
    uint16_t slot = i;

    bool occ;
    if (size == 0) {
      occ = false;
    } else if (ringHead + size <= cap) {
      occ = (slot >= ringHead) && (slot < (uint16_t)(ringHead + size));
    } else {
      uint16_t end = (uint16_t)((ringHead + size) % cap);
      occ = (slot >= ringHead) || (slot < end);
    }

    if (slot == ringHead && slot == ringTail)      DEBUG_PRINT("HT");
    else if (slot == ringHead)                     DEBUG_PRINT("H");
    else if (slot == ringTail)                     DEBUG_PRINT("T");
    else                                           DEBUG_PRINT(occ ? "■" : "·");
  }
  DEBUG_PRINTLN();
}

// ----------------------------------------------------------------------------
//  printMoRecordSummary
//
//  Prints a single record in human-readable form (oldest-first).
//  Scales values using the union’s conventions.
// ----------------------------------------------------------------------------
static void decodeMoRecord(const uint8_t* src, SBD_MO_MESSAGE& out) {
  memcpy(out.bytes, src, SBD_MSG_SIZE);
}

void printMoRecordSummary(uint16_t logicalIndex, const uint8_t* raw) {
  SBD_MO_MESSAGE rec{};
  decodeMoRecord(raw, rec);

  DEBUG_PRINT("[Queue] #");
  DEBUG_PRINT(logicalIndex);
  DEBUG_PRINT(" | unix: ");
  DEBUG_PRINT(rec.unixtime);
  DEBUG_PRINT(" | lat: ");
  DEBUG_PRINT((float)rec.latitude / 1000000.0f);
  DEBUG_PRINT(" | lon: ");
  DEBUG_PRINT((float)rec.longitude / 1000000.0f);
  DEBUG_PRINT(" | Vbat: ");
  DEBUG_PRINT((float)rec.voltage / 100.0f);
  DEBUG_PRINT(" V | sats: ");
  DEBUG_PRINT(rec.satellites);
  DEBUG_PRINT(" | hdop: ");
  DEBUG_PRINT((float)rec.hdop / 100.0f);
  DEBUG_PRINT(" | iter: ");
  DEBUG_PRINT(rec.iterationCounter);
  DEBUG_PRINT(" | txDur: ");
  DEBUG_PRINT(rec.transmitDuration);
  DEBUG_PRINT(" s | txRC: ");
  DEBUG_PRINT(rec.transmitStatus);
  DEBUG_PRINTLN();
}

// ----------------------------------------------------------------------------
//  printRingBufferPreview
//
//  Prints the first N (oldest) records as readable summaries.
// ----------------------------------------------------------------------------
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

// ----------------------------------------------------------------------------
//  printNextTransmitPreview

//  Shows exactly which messages will go in the next MO window (up to 10).
// ----------------------------------------------------------------------------
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

// Dump the full 340-byte MO window (prints all bytes, including zeros)
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

  // Sanity log (derived from your constants)
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
//  Copies up to SBD_MAX_MSGS oldest records from the ring buffer into moSbdBuffer.
//  - outBytes: number of bytes staged for this session
//  - returns:  how many messages were staged (0..SBD_MAX_MSGS) 
// ----------------------------------------------------------------------------
uint16_t buildMoPayloadFromQueue(uint8_t* dst, size_t dstBytes, size_t* outBytes) {
  const uint16_t queued = ringBufferSize();
  if (queued == 0) { *outBytes = 0; return 0; }

  // Limit by modem window
  uint16_t take = (queued > SBD_MAX_MSGS) ? SBD_MAX_MSGS : queued;
  size_t needBytes = (size_t)take * (size_t)SBD_MSG_SIZE;

  // Defensive clamp (should match SBD_BUF_BYTES)
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
// Write one message into the queue (called each sampling interval)
// 
//  - Updates dynamic fields in the message union.
//  - Serializes and enqueues exactly SBD_MSG_SIZE bytes into our ring buffer.
//  - Increments the per-window transmitCounter (the scheduler uses this).
  
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

  // Populate dynamic fields
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
  printRingBufferMap();           // visual occupancy map (optional)
  // printRingBufferPreview(3);   // preview oldest N (optional)
}

// ----------------------------------------------------------------------------
// Transmit oldest messages: send up to SBD_MAX_MSGS every TX window
//  - If there are queued messages, stage up to SBD_MAX_MSGS oldest records
//    into the Iridium MO buffer and attempt a send/receive.
//  - On success: pop N that were sent, reset per-window counter.
//  - On failure: keep everything; we’ll try again next window (no data loss).
// ----------------------------------------------------------------------------
void transmitData() {
  // If nothing queued, just reset the per-window counter and exit
  if (ringBufferEmpty()) {
    DEBUG_PRINTLN("[Iridium] Info: No queued messages to send.");
    transmitCounter = 0;
    return;
  }

  // Stage payload (up to 10 messages = 340 bytes)
  size_t moBytes = 0;
  memset(moSbdBuffer, 0x00, sizeof(moSbdBuffer));
  const uint16_t stagedMsgs = buildMoPayloadFromQueue(moSbdBuffer, sizeof(moSbdBuffer), &moBytes);

  DEBUG_PRINT("[Iridium] Info: Staging ");
  DEBUG_PRINT(stagedMsgs);
  DEBUG_PRINT(" messages (");
  DEBUG_PRINT(moBytes);
  DEBUG_PRINTLN(" bytes) for TX.");

  ("transmitData(pre)");
  printRingBufferStatus("transmitData(pre)");
  printNextTransmitPreview();                 // human-readable batch
  printMoWindowHex(moSbdBuffer, SBD_BUF_BYTES); // full 340B hex dump

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
  } else {
    // Prepare MT buffer
    memset(mtSbdBuffer, 0x00, sizeof(mtSbdBuffer));
    mtSbdBufferSize = sizeof(mtSbdBuffer);

    DEBUG_PRINTLN("[Iridium] Info: Attempting to transmit message...");
    rc = modem.sendReceiveSBDBinary(
           moSbdBuffer,
           moBytes,
           mtSbdBuffer,
           mtSbdBufferSize);

    if (rc == ISBD_SUCCESS) {
      DEBUG_PRINTLN("[Iridium] Info: MO-SBD message transmission successful!");
      blinkLed(10, 250);

      // Remove only what we sent
      ringBufferPop(stagedMsgs);

      // Reset per-window counter
      transmitCounter = 0; // start a fresh window

      // Check if a Mobile Terminated (MT) message was received
      if (mtSbdBufferSize == 7) {
        for (size_t i = 0; i < mtSbdBufferSize; ++i) {
          mtSbdMessage.bytes[i] = mtSbdBuffer[i];
        }
        printMtSbdBuffer();
        printMtSbd();

        if (validateMtSbdMessage(mtSbdMessage)) {
          DEBUG_PRINTLN("[Iridium] Info: All received values within accepted ranges.");
          // Apply any updated settings, even if not used here
          alarmMode = mtSbdMessage.alarmMode;
          alarmIntervalDay = mtSbdMessage.alarmIntervalDay;
          alarmIntervalHour = mtSbdMessage.alarmIntervalHour;
          alarmIntervalMinute = mtSbdMessage.alarmIntervalMinute;
          transmitInterval = mtSbdMessage.transmitInterval;
          // transmitReattempts is accepted but not used by this strategy
          resetFlag = mtSbdMessage.resetFlag;
          DEBUG_PRINTLN("[Iridium] Info: System parameters updated from MT-SBD message.");
        } else {
          DEBUG_PRINTLN("[Iridium] Warning: Received values exceed accepted range!");
        }
      }
    } else {
      DEBUG_PRINTLN("[Iridium] Warning: Transmission failed.");
      printIridiumError(rc);
      blinkLed(5, 1000);

      // Keep queued messages; wait for next transmit window
      transmitCounter = 0; // wait a full interval before next attempt
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
    DEBUG_PRINTLN("[Iridium] Info: Modem put to sleep successfullly.");
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
    while (true) ; // Wait for WDT reset
  }
}

// ----------------------------------------------------------------------------
// Validation, callbacks, sleep pin, and error helpers
// ----------------------------------------------------------------------------
bool validateMtSbdMessage(const SBD_MT_MESSAGE& msg) {
  return (
    msg.alarmMode <= 2
    && msg.alarmIntervalDay <= 31
    && msg.alarmIntervalHour >= 0 && msg.alarmIntervalHour < 24  // allow 0
    && msg.alarmIntervalMinute < 60
    && msg.transmitInterval >= 1 && msg.transmitInterval <= 10
    && msg.transmitReattempts <= 10
    && (msg.resetFlag == 0 || msg.resetFlag == 255));
}

// Non-blocking callback during Iridium transmissions (keeps WDT happy)
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
void ISBDConsoleCallback(IridiumSBD* /*device*/, char c) { DEBUG_WRITE(c); }
void ISBDDiagsCallback(IridiumSBD* /*device*/, char c)   { DEBUG_WRITE(c); }
#endif

// Inverted sleep control via N-MOSFET (keep your working version)
void IridiumSBD::setSleepPin(uint8_t enable) {
  if (enable == HIGH) {
    digitalWrite(this->sleepPin, LOW);  // LOW = awake (inverted by N-MOSFET)
    diagprint(F("Modem is awake.\r\n"));
  } else {
    digitalWrite(this->sleepPin, HIGH); // HIGH = asleep (inverted by N-MOSFET)
    diagprint(F("Modem is asleep.\r\n"));
  }
}

// ----------------------------------------------------------------------------
// Error helpers
// ----------------------------------------------------------------------------
void printIridiumError(int code) {
  DEBUG_PRINT("[Iridium] Error ");
  DEBUG_PRINT(code);
  DEBUG_PRINT(": ");
  DEBUG_PRINTLN(iridiumErrorDescription(code));
}

const char* iridiumErrorDescription(int code) {
  switch (code) {
    case ISBD_SUCCESS:            return "Success.";
    case ISBD_ALREADY_AWAKE:      return "Modem already awake.";
    case ISBD_SERIAL_FAILURE:     return "Serial communication failure.";
    case ISBD_PROTOCOL_ERROR:     return "AT command protocol error.";
    case ISBD_CANCELLED:          return "Operation cancelled.";
    case ISBD_NO_MODEM_DETECTED:  return "No modem detected. Check wiring.";
    case ISBD_SBDIX_FATAL_ERROR:  return "Fatal error during SBDIX.";
    case ISBD_SENDRECEIVE_TIMEOUT:return "Send/receive timed out.";
    case ISBD_RX_OVERFLOW:        return "Receive buffer overflow.";
    case ISBD_REENTRANT:          return "Reentrant call detected.";
    case ISBD_IS_ASLEEP:          return "Modem is asleep.";
    case ISBD_NO_SLEEP_PIN:       return "Sleep pin not configured.";
    case ISBD_NO_NETWORK:         return "No network available.";
    case ISBD_MSG_TOO_LONG:       return "Message too long.";
    default:                      return "Unknown error.";
  }
}
