/*
  GNSS Module

  This module configures and reads the GNSS receiver, parses incoming
  NMEA sentences using TinyGPS++, and acquires a robust position fix
  using component median filtering of multiple accepted samples. It also
  synchronizes the RTC using GNSS time when available and updates the
  moSbdMessage structure with GNSS position and quality information.
*/

// ----------------------------------------------------------------------------
// Tuning parameters
// ----------------------------------------------------------------------------
static const uint16_t HDOP_PREFERRED_MAX = 250;   // <= 2.50 HDOP for preferred fixes
static const uint8_t SATS_PREFERRED_MIN = 5;      // Minimum satellites for preferred fixes
static const uint32_t FIELD_STALE_MS_MAX = 1500;  // NMEA field freshness gate (ms)

static const uint8_t GNSS_PREFERRED_TARGET = 15;  // Exit early if this many preferred fixes collected
static const uint8_t GNSS_SAMPLE_TARGET = 30;     // Maximum acceptable fixes to collect
static const uint8_t GNSS_SAMPLE_MIN = 1;         // Minimum fixes needed to report a position

// No-fix sentinels (avoid 0,0 ambiguity in payload)
static const int32_t GNSS_LATLON_NOVALUE = -999999999L;
static const uint16_t GNSS_HDOP_NOVALUE = 0xFFFF;
static const uint8_t GNSS_SATS_NOVALUE = 0xFF;

// ----------------------------------------------------------------------------
// PMTK helpers (checksum computed for any payload)
// ----------------------------------------------------------------------------
static void sendPMTK(const char* payload) {
  uint8_t cs = 0;
  for (const char* p = payload; *p; ++p) cs ^= (uint8_t)(*p);
  char line[64];
  snprintf(line, sizeof(line), "$%s*%02X", payload, cs);
  GNSS_PORT.println(line);
  myDelay(100);
}

static inline void setUpdateRateMs(uint16_t ms) {
  char buf[24];
  snprintf(buf, sizeof(buf), "PMTK220,%u", (unsigned)ms);
  sendPMTK(buf);
}

static inline void setSentenceOutput_GGA_RMC() {
  // GGA=1, RMC=1, everything else 0
  sendPMTK("PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0");
}

// ----------------------------------------------------------------------------
// GNSS configuration
// ----------------------------------------------------------------------------
void configureGnss() {
  // Set NMEA update rate
  setUpdateRateMs(1000);  // Set rate to 1 Hz (1000 ms)

  // Set NMEA sentence output frequencies to GGA and RMC
  setSentenceOutput_GGA_RMC();

  // Optional antenna update configuration
  // sendPMTK("PGCMD,33,0"); // Disable antenna status messages
}

// ----------------------------------------------------------------------------
// Base validity check shared by both fix tiers.
// Ensures all required fields are present and fresh before any quality
// thresholds are applied. An acceptable fix is any fix that passes this
// check — the receiver's own validity judgement is the only gate.
// ----------------------------------------------------------------------------
static bool isFreshValidFix() {
  if (!gnss.location.isValid() || gnss.location.age() > FIELD_STALE_MS_MAX) return false;
  if (!gnss.time.isValid() || gnss.time.age() > FIELD_STALE_MS_MAX) return false;
  if (!gnss.date.isValid() || gnss.date.age() > FIELD_STALE_MS_MAX) return false;
  if (!gnss.hdop.isValid() || gnss.hdop.age() > FIELD_STALE_MS_MAX) return false;
  if (!gnss.satellites.isValid() || gnss.satellites.age() > FIELD_STALE_MS_MAX) return false;
  return true;
}

// ----------------------------------------------------------------------------
// Checks for a preferred fix: fresh and valid, with low HDOP and high
// satellite count. These drive toward early exit at GNSS_PREFERRED_TARGET
// and produce the highest quality median. Also stored in the acceptable
// buffer since preferred is a subset of acceptable.
// ----------------------------------------------------------------------------
static bool isPreferredFix() {
  if (!isFreshValidFix()) return false;
  if (gnss.satellites.value() < SATS_PREFERRED_MIN) return false;
  if (gnss.hdop.value() > HDOP_PREFERRED_MAX) return false;
  return true;
}

// ----------------------------------------------------------------------------
// Component median helpers.
// ----------------------------------------------------------------------------
static double medianDouble(double values[], uint8_t count) {
  double sorted[GNSS_SAMPLE_TARGET];

  for (uint8_t i = 0; i < count; i++) {
    sorted[i] = values[i];
  }

  for (uint8_t i = 1; i < count; i++) {
    double key = sorted[i];
    int8_t j = i - 1;

    while (j >= 0 && sorted[j] > key) {
      sorted[j + 1] = sorted[j];
      j--;
    }

    sorted[j + 1] = key;
  }

  if (count % 2 == 1) {
    return sorted[count / 2];
  }

  return (sorted[(count / 2) - 1] + sorted[count / 2]) / 2.0;
}

static uint16_t medianUint16(uint16_t values[], uint8_t count) {
  uint16_t sorted[GNSS_SAMPLE_TARGET];

  for (uint8_t i = 0; i < count; i++) {
    sorted[i] = values[i];
  }

  for (uint8_t i = 1; i < count; i++) {
    uint16_t key = sorted[i];
    int8_t j = i - 1;

    while (j >= 0 && sorted[j] > key) {
      sorted[j + 1] = sorted[j];
      j--;
    }

    sorted[j + 1] = key;
  }

  if (count % 2 == 1) {
    return sorted[count / 2];
  }

  return (uint16_t)(((uint32_t)sorted[(count / 2) - 1] + sorted[count / 2]) / 2);
}

static uint8_t medianUint8(uint8_t values[], uint8_t count) {
  uint8_t sorted[GNSS_SAMPLE_TARGET];

  for (uint8_t i = 0; i < count; i++) {
    sorted[i] = values[i];
  }

  for (uint8_t i = 1; i < count; i++) {
    uint8_t key = sorted[i];
    int8_t j = i - 1;

    while (j >= 0 && sorted[j] > key) {
      sorted[j + 1] = sorted[j];
      j--;
    }

    sorted[j + 1] = key;
  }

  if (count % 2 == 1) {
    return sorted[count / 2];
  }

  return (uint8_t)(((uint16_t)sorted[(count / 2) - 1] + sorted[count / 2]) / 2);
}

// ----------------------------------------------------------------------------
// Computes and stores a median fix from the sample buffer.
// Called from all exit paths to avoid duplicating the median computation
// and debug output.
// ----------------------------------------------------------------------------
static void computeMedianFix(double latSamples[], double lngSamples[],
                             uint8_t satsSamples[], uint16_t hdopSamples[],
                             uint8_t count, time_t epoch) {
  latitude = medianDouble(latSamples, count);
  longitude = medianDouble(lngSamples, count);
  satellites = medianUint8(satsSamples, count);
  hdop = medianUint16(hdopSamples, count);
  gnssEpoch = epoch;

  DEBUG_PRINT("[GNSS] Info: Median fix calculated from ");
  DEBUG_PRINT(count);
  DEBUG_PRINTLN(" samples.");
  DEBUG_PRINT("[GNSS] Info: Median latitude = ");
  DEBUG_PRINTLN_DEC(latitude, 6);
  DEBUG_PRINT("[GNSS] Info: Median longitude = ");
  DEBUG_PRINTLN_DEC(longitude, 6);
  DEBUG_PRINT("[GNSS] Info: Median satellites = ");
  DEBUG_PRINTLN(satellites);
  DEBUG_PRINT("[GNSS] Info: Median HDOP = ");
  DEBUG_PRINTLN_DEC((float)hdop / 100.0f, 2);
}

// ----------------------------------------------------------------------------
// Reads the GNSS receiver.
// If a valid fix is found, sync the RTC if newer than the current unixtime,
// update moSbdMessage with GNSS data, and log RTC drift.
// ----------------------------------------------------------------------------
void readGnss() {
  // Start execution timer
  uint32_t startTime = millis();

  // Clear flags
  bool fixFound = false;
  bool charsSeen = false;

  // Preferred buffer: high-quality fixes only (HDOP <= HDOP_PREFERRED_MAX,
  // sats >= SATS_PREFERRED_MIN). Used for the median if enough are collected.
  double preferredLatSamples[GNSS_PREFERRED_TARGET];
  double preferredLngSamples[GNSS_PREFERRED_TARGET];
  uint8_t preferredSatsSamples[GNSS_PREFERRED_TARGET];
  uint16_t preferredHdopSamples[GNSS_PREFERRED_TARGET];
  uint8_t preferredCount = 0;

  // Acceptable buffer: any fix the receiver considers valid and fresh,
  // including preferred ones. Used as fallback if preferred count is
  // insufficient.
  double acceptableLatSamples[GNSS_SAMPLE_TARGET];
  double acceptableLngSamples[GNSS_SAMPLE_TARGET];
  uint8_t acceptableSatsSamples[GNSS_SAMPLE_TARGET];
  uint16_t acceptableHdopSamples[GNSS_SAMPLE_TARGET];
  uint8_t acceptableCount = 0;

  tmElements_t tm;
  time_t lastEpoch = 0;

  // Enable power to GNSS
  enableGnssPower();

  DEBUG_PRINTLN("[GNSS] Info: Beginning to listen for GNSS traffic...");

  // Initialize GNSS serial port
  GNSS_PORT.begin(9600);
  myDelay(1000);

  // Configure GNSS
  configureGnss();

  // Look for GNSS signal up to gnssTimeout seconds
  const uint32_t timeoutMs = (uint32_t)gnssTimeout * 1000UL;

  while (!fixFound && (millis() - startTime < timeoutMs)) {
    // Feed TinyGPS++
    while (GNSS_PORT.available()) {
      charsSeen = true;
      char c = GNSS_PORT.read();

#if DEBUG_GNSS
      Serial.write(c);
#endif

      gnss.encode(c);
    }

    // ------------------------------------------------------------------------
    // Only evaluate fixes when a new GGA update is available.
    // HDOP is provided by GGA, so this avoids repeatedly checking the same
    // parsed TinyGPS++ values between NMEA sentences.
    // ------------------------------------------------------------------------
    if (gnss.hdop.isUpdated()) {

      bool preferred = isPreferredFix();
      bool acceptable = !preferred && isFreshValidFix();

      if (preferred || acceptable) {

        // Capture time elements once for both buffers
        tm.Hour = gnss.time.hour();
        tm.Minute = gnss.time.minute();
        tm.Second = gnss.time.second();
        tm.Day = gnss.date.day();
        tm.Month = gnss.date.month();
        tm.Year = gnss.date.year() - 1970;
        lastEpoch = makeTime(tm);

        // Store in acceptable buffer — receives all fixes including preferred
        if (acceptableCount < GNSS_SAMPLE_TARGET) {
          acceptableLatSamples[acceptableCount] = gnss.location.lat();
          acceptableLngSamples[acceptableCount] = gnss.location.lng();
          acceptableSatsSamples[acceptableCount] = gnss.satellites.value();
          acceptableHdopSamples[acceptableCount] = gnss.hdop.value();
          acceptableCount++;
        }

        // Additionally store in preferred buffer if it qualifies
        if (preferred && preferredCount < GNSS_PREFERRED_TARGET) {
          preferredLatSamples[preferredCount] = gnss.location.lat();
          preferredLngSamples[preferredCount] = gnss.location.lng();
          preferredSatsSamples[preferredCount] = gnss.satellites.value();
          preferredHdopSamples[preferredCount] = gnss.hdop.value();
          preferredCount++;
        }

        DEBUG_PRINT("[GNSS] Info: Sample ");
        DEBUG_PRINT(acceptableCount);
        DEBUG_PRINT("/");
        DEBUG_PRINT(GNSS_SAMPLE_TARGET);
        DEBUG_PRINT(preferred ? " [preferred] preferred: " : " [acceptable] preferred: ");
        DEBUG_PRINT(preferredCount);
        DEBUG_PRINT("/");
        DEBUG_PRINTLN(GNSS_PREFERRED_TARGET);

        // Exit early if enough preferred fixes have been collected —
        // preferred buffer contains only high-quality uncontaminated data
        if (preferredCount >= GNSS_PREFERRED_TARGET) {
          computeMedianFix(preferredLatSamples, preferredLngSamples,
                           preferredSatsSamples, preferredHdopSamples,
                           preferredCount, lastEpoch);
          DEBUG_PRINTLN("[GNSS] Info: Preferred fix threshold reached. Exiting early.");
          fixFound = true;
        }

        // Exit if the acceptable buffer is full — use preferred if any exist,
        // otherwise fall back to the full acceptable buffer
        if (!fixFound && acceptableCount >= GNSS_SAMPLE_TARGET) {
          if (preferredCount >= GNSS_SAMPLE_MIN) {
            computeMedianFix(preferredLatSamples, preferredLngSamples,
                             preferredSatsSamples, preferredHdopSamples,
                             preferredCount, lastEpoch);
            DEBUG_PRINTLN("[GNSS] Info: Acceptable buffer full. Using preferred median.");
          } else {
            computeMedianFix(acceptableLatSamples, acceptableLngSamples,
                             acceptableSatsSamples, acceptableHdopSamples,
                             acceptableCount, lastEpoch);
            DEBUG_PRINTLN("[GNSS] Info: Acceptable buffer full. Using acceptable median.");
          }
          fixFound = true;
        }
      }
    }

    // Call callback during acquisition
    ISBDCallback();

    // If no data after ~5 seconds, break
    if ((millis() - startTime) > 5000UL && !charsSeen) {
      DEBUG_PRINTLN("[GNSS] Warning: No NMEA characters received after 5 seconds. Check GNSS wiring and baud rate.");
      break;
    }
  }

  // Timeout reached — use whatever was collected, preferring the preferred
  // buffer if it has any samples, otherwise using the acceptable buffer
  if (!fixFound) {
    if (preferredCount >= GNSS_SAMPLE_MIN) {
      computeMedianFix(preferredLatSamples, preferredLngSamples,
                       preferredSatsSamples, preferredHdopSamples,
                       preferredCount, lastEpoch);
      DEBUG_PRINTLN("[GNSS] Info: Timeout reached. Using preferred median.");
      fixFound = true;
    } else if (acceptableCount >= GNSS_SAMPLE_MIN) {
      computeMedianFix(acceptableLatSamples, acceptableLngSamples,
                       acceptableSatsSamples, acceptableHdopSamples,
                       acceptableCount, lastEpoch);
      DEBUG_PRINTLN("[GNSS] Info: Timeout reached. Using acceptable median.");
      fixFound = true;
    }
  }

  if (fixFound) {

    // Synchronize RTC from GNSS
    DEBUG_PRINTLN("[RTC] Info: Syncing RTC from median fix...");
    syncRtcFromGnss(gnssEpoch);

    // Store GNSS data in moSbdMessage
    moSbdMessage.latitude = (int32_t)lround(latitude * 1000000.0);
    moSbdMessage.longitude = (int32_t)lround(longitude * 1000000.0);
    moSbdMessage.satellites = satellites;
    moSbdMessage.hdop = (uint16_t)hdop;  // Keep TinyGPS++ raw units

    DEBUG_PRINT("[GNSS] Info: RTC drift = ");
    DEBUG_PRINT(rtcDrift);
    DEBUG_PRINTLN(" seconds.");

    blinkLed(5, 100);
  } else {
    DEBUG_PRINTLN("[GNSS] Warning: No GNSS fix found!");

    // Explicit "no-fix" payload
    moSbdMessage.latitude = GNSS_LATLON_NOVALUE;
    moSbdMessage.longitude = GNSS_LATLON_NOVALUE;
    moSbdMessage.satellites = GNSS_SATS_NOVALUE;
    moSbdMessage.hdop = GNSS_HDOP_NOVALUE;

    // Also clear globals on no-fix (so any downstream logic can detect it)
    latitude = 0.0f;
    longitude = 0.0f;
    satellites = 0;
    hdop = 0;
  }

  // Close GNSS Serial port
  GNSS_PORT.end();

  // Disable power to GNSS
  disableGnssPower();

  // Record elapsed execution time
  timer.readGnss = millis() - startTime;
}