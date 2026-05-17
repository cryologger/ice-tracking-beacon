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
static const uint16_t HDOP_GOOD_MAX = 250;        // <= 2.50 for accepted samples
static const uint8_t SATS_GOOD_MIN = 5;           // Prefer stable 3D
static const uint32_t FIELD_STALE_MS_MAX = 1500;  // NMEA field freshness gate

static const uint8_t GNSS_SAMPLE_TARGET = 30;  // Target number of fixes to collect for median
static const uint8_t GNSS_SAMPLE_MIN = 10;     // Minimum fixes needed to use median

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
  setUpdateRateMs(1000); // Set rate to 1 Hz (1000 ms)

  // Set NMEA sentence output frequencies to GGA and RMC
  setSentenceOutput_GGA_RMC();

  // Optional antenna update configuration
  // sendPMTK("PGCMD,33,0"); // Disable antenna status messages
}

// ----------------------------------------------------------------------------
// Checks for an acceptable GNSS fix to add to the median sample buffer.
// ----------------------------------------------------------------------------
static bool isGoodFix() {
  if (!gnss.location.isValid() || gnss.location.age() > FIELD_STALE_MS_MAX) return false;
  if (!gnss.time.isValid() || gnss.time.age() > FIELD_STALE_MS_MAX) return false;
  if (!gnss.date.isValid() || gnss.date.age() > FIELD_STALE_MS_MAX) return false;
  if (!gnss.hdop.isValid() || !gnss.satellites.isValid()) return false;

  if (gnss.satellites.value() < SATS_GOOD_MIN) return false;
  if (gnss.hdop.value() > HDOP_GOOD_MAX) return false;

  return true;
}

// ----------------------------------------------------------------------------
// “Any valid” fix used as a fallback if not enough samples collected.
// ----------------------------------------------------------------------------
static bool isValidFix() {
  return (
    gnss.location.isValid() && gnss.location.age() < FIELD_STALE_MS_MAX
    && gnss.time.isValid() && gnss.time.age() < FIELD_STALE_MS_MAX
    && gnss.date.isValid() && gnss.date.age() < FIELD_STALE_MS_MAX
    && gnss.satellites.isValid() && gnss.satellites.value() > 0);
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

  double latSamples[GNSS_SAMPLE_TARGET];
  double lngSamples[GNSS_SAMPLE_TARGET];
  uint8_t satsSamples[GNSS_SAMPLE_TARGET];
  uint16_t hdopSamples[GNSS_SAMPLE_TARGET];

  uint8_t sampleCount = 0;
  time_t lastGoodEpoch = 0;
  bool haveFallback = false;
  double fbLat = 0.0, fbLng = 0.0;
  uint8_t fbSats = 0;
  uint16_t fbHdop = 9999;
  time_t fbEpoch = 0;

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

      // Maintain a fallback fix in case we do not collect enough good samples.
      if (isValidFix()) {
        uint16_t h = gnss.hdop.isValid() ? gnss.hdop.value() : 9999;

        if (!haveFallback || h < fbHdop) {
          haveFallback = true;
          fbLat = gnss.location.lat();
          fbLng = gnss.location.lng();
          fbSats = gnss.satellites.isValid() ? gnss.satellites.value() : 0;
          fbHdop = h;

          tm.Hour = gnss.time.hour();
          tm.Minute = gnss.time.minute();
          tm.Second = gnss.time.second();
          tm.Day = gnss.date.day();
          tm.Month = gnss.date.month();
          tm.Year = gnss.date.year() - 1970;
          fbEpoch = makeTime(tm);
        }
      }

      // Add accepted fixes to the median sample buffer.
      if (isGoodFix() && sampleCount < GNSS_SAMPLE_TARGET) {
        latSamples[sampleCount] = gnss.location.lat();
        lngSamples[sampleCount] = gnss.location.lng();
        satsSamples[sampleCount] = gnss.satellites.value();
        hdopSamples[sampleCount] = gnss.hdop.value();

        tm.Hour = gnss.time.hour();
        tm.Minute = gnss.time.minute();
        tm.Second = gnss.time.second();
        tm.Day = gnss.date.day();
        tm.Month = gnss.date.month();
        tm.Year = gnss.date.year() - 1970;
        lastGoodEpoch = makeTime(tm);

        sampleCount++;

        DEBUG_PRINT("[GNSS] Info: GNSS sample ");
        DEBUG_PRINT(sampleCount);
        DEBUG_PRINT("/");
        DEBUG_PRINTLN(GNSS_SAMPLE_TARGET);

        if (sampleCount >= GNSS_SAMPLE_TARGET) {
          latitude = medianDouble(latSamples, sampleCount);
          longitude = medianDouble(lngSamples, sampleCount);
          satellites = medianUint8(satsSamples, sampleCount);
          hdop = medianUint16(hdopSamples, sampleCount);
          gnssEpoch = lastGoodEpoch;

          fixFound = true;

          DEBUG_PRINT("[GNSS] Info: Median fix calculated from ");
          DEBUG_PRINT(sampleCount);
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

  // If timeout was reached with enough samples but not the full target, use partial median
  if (!fixFound && sampleCount >= GNSS_SAMPLE_MIN) {
    latitude = medianDouble(latSamples, sampleCount);
    longitude = medianDouble(lngSamples, sampleCount);
    satellites = medianUint8(satsSamples, sampleCount);
    hdop = medianUint16(hdopSamples, sampleCount);
    gnssEpoch = lastGoodEpoch;
    fixFound = true;

    DEBUG_PRINT("[GNSS] Info: Partial median fix from ");
    DEBUG_PRINT(sampleCount);
    DEBUG_PRINTLN(" samples (timeout reached).");
    DEBUG_PRINT("[GNSS] Info: Partial median latitude = ");
    DEBUG_PRINTLN_DEC(latitude, 6);
    DEBUG_PRINT("[GNSS] Info: Partial median longitude = ");
    DEBUG_PRINTLN_DEC(longitude, 6);
    DEBUG_PRINT("[GNSS] Info: Partial median satellites = ");
    DEBUG_PRINTLN(satellites);
    DEBUG_PRINT("[GNSS] Info: Partial median HDOP = ");
    DEBUG_PRINTLN_DEC((float)hdop / 100.0f, 2);
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
  } else if (haveFallback) {
    DEBUG_PRINTLN("[GNSS] Info: Using best valid fix before timeout. Not enough samples for median.");

    // Sync RTC from fallback if newer
    DEBUG_PRINTLN("[RTC] Info: Syncing RTC from fallback fix...");
    syncRtcFromGnss(fbEpoch);

    // Set fallback
    latitude = fbLat;
    longitude = fbLng;
    satellites = fbSats;
    hdop = fbHdop;

    moSbdMessage.latitude = (int32_t)lround(latitude * 1000000.0);
    moSbdMessage.longitude = (int32_t)lround(longitude * 1000000.0);
    moSbdMessage.satellites = satellites;
    moSbdMessage.hdop = (uint16_t)hdop;
  } else {
    DEBUG_PRINTLN("[GNSS] Warning: No GNSS fix found!");

    // Explicit “no-fix” payload
    moSbdMessage.latitude = GNSS_LATLON_NOVALUE;
    moSbdMessage.longitude = GNSS_LATLON_NOVALUE;
    moSbdMessage.satellites = GNSS_SATS_NOVALUE;
    moSbdMessage.hdop = GNSS_HDOP_NOVALUE;

    // Also clear globals on no-fix (so any downstream logic can detect it)
    latitude = 0.0;
    longitude = 0.0;
    satellites = 0;
    hdop = 0.0;
  }

  // Close GNSS Serial port
  GNSS_PORT.end();

  // Disable power to GNSS
  disableGnssPower();

  // Record elapsed execution time
  timer.readGnss = millis() - startTime;
}