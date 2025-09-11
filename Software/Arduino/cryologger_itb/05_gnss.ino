/*
  GNSS Module

  This module handles reading GNSS data from an attached receiver,
  parsing NMEA sentences via TinyGPS++. It attempts to acquire a valid
  fix within a user-defined timeout, synchronizes the RTC if the GNSS
  time is newer, and updates the moSbdMessage structure with location
  and satellite data.
*/

// ----------------------------------------------------------------------------
// Tuning parameters
// ----------------------------------------------------------------------------
static const uint16_t HDOP_GOOD_MAX = 250;        // <= 2.50 for "good" gate
static const uint8_t SATS_GOOD_MIN = 5;           // Prefer stable 3D
static const uint8_t REQUIRED_CONSEC_FIXES = 20;  // 10 consecutive GGA fixes
static const uint32_t FIELD_STALE_MS_MAX = 1500;  // NMEA field freshness gate

// Quality window after hitting the 10-good gate
static const uint32_t EXTRA_QUALITY_WINDOW_MS = 10000;  // Keep listening up to +10 s
static const uint16_t HDOP_EARLY_EXIT_MAX = 120;        // Exit early if <= 1.20 seen

// No-fix sentinels (avoid 0,0 ambiguity in payload)
static const int32_t GNSS_LATLON_NOVALUE = -999999999L;
static const uint16_t GNSS_HDOP_NOVALUE = 0xFFFF;
static const uint8_t GNSS_SATS_NOVALUE = 0;

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
  // Set NMEA update rate (2 Hz)
  setUpdateRateMs(500);

  // Set NMEA sentence output frequencies to GGA and RMC
  setSentenceOutput_GGA_RMC();

  // Optional antenna update configuration
  // sendPMTK("PGCMD,33,0"); // Disable antenna status messages
}

// ----------------------------------------------------------------------------
// Checks for “good” GNSS fix for the purpose of the consecutive threshold.
// Also gates on NMEA field freshness to avoid re-counting stale data.
// ----------------------------------------------------------------------------
static bool isGoodFixForConsecutive() {
  if (!gnss.location.isValid() || gnss.location.age() > FIELD_STALE_MS_MAX) return false;
  if (!gnss.time.isValid() || gnss.time.age() > FIELD_STALE_MS_MAX) return false;
  if (!gnss.date.isValid() || gnss.date.age() > FIELD_STALE_MS_MAX) return false;
  if (!gnss.hdop.isValid() || !gnss.satellites.isValid()) return false;

  const uint16_t hdopRaw = gnss.hdop.value();
  const uint8_t sats = gnss.satellites.value();

  if (sats < SATS_GOOD_MIN) return false;
  if (hdopRaw > HDOP_GOOD_MAX) return false;

  return true;
}

// ----------------------------------------------------------------------------
// “Any valid” fix (looser) used for a fallback if we never hit good gate.
// ----------------------------------------------------------------------------
static bool isValidFixLoose() {
  return (
    gnss.location.isValid() && gnss.location.age() < FIELD_STALE_MS_MAX
    && gnss.time.isValid() && gnss.time.age() < FIELD_STALE_MS_MAX
    && gnss.date.isValid() && gnss.date.age() < FIELD_STALE_MS_MAX
    && gnss.satellites.isValid() && gnss.satellites.value() > 0);
}

// ----------------------------------------------------------------------------
// Reads the GNSS receiver.
// If a valid fix is found, sync the RTC if newer than the current unixtime,
// update moSbdMessage with GNSS data, and log RTC drift.
// ----------------------------------------------------------------------------
void readGnss() {
  // Start execution timer
  uint32_t startTime = millis();

  // Clear flags.
  bool fixFound = false;
  bool charsSeen = false;

  // Consecutive “good” (GGA) fixes counter
  uint8_t fixCounter = 0;

  // Best “good” seen (for the quality window)
  bool haveBestGood = false;
  double bestLat = 0.0, bestLng = 0.0;
  uint8_t bestSats = 0;
  uint16_t bestHdop = 9999;
  time_t bestEpoch = 0;

  // Best “loose-valid” fallback seen during this session
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
    // Count only on NEW GGA updates to avoid RMC double-count
    // ------------------------------------------------------------------------
    if (gnss.hdop.isUpdated()) {

      // Maintain a best “loose-valid” fallback by lowest HDOP (sats > 0, fields fresh)
      if (isValidFixLoose()) {
        uint16_t h = gnss.hdop.isValid() ? gnss.hdop.value() : 9999;
        if (!haveFallback || h < fbHdop) {
          haveFallback = true;
          fbLat = gnss.location.lat();
          fbLng = gnss.location.lng();
          fbSats = gnss.satellites.value();
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

      // Enforce “X consecutive good fixes” using GGA-only cadence
      if (isGoodFixForConsecutive()) {
        fixCounter++;

        // Track best “good” so far (used by the quality window)
        uint16_t h = gnss.hdop.value();
        if (!haveBestGood || h < bestHdop) {
          haveBestGood = true;
          bestLat = gnss.location.lat();
          bestLng = gnss.location.lng();
          bestSats = gnss.satellites.value();
          bestHdop = h;

          tm.Hour = gnss.time.hour();
          tm.Minute = gnss.time.minute();
          tm.Second = gnss.time.second();
          tm.Day = gnss.date.day();
          tm.Month = gnss.date.month();
          tm.Year = gnss.date.year() - 1970;
          bestEpoch = makeTime(tm);
        }
      } else {
        fixCounter = 0;  // Reset streak on any non-good GGA
      }

      // When we’ve reached the gate, run the short quality window
      if (fixCounter >= REQUIRED_CONSEC_FIXES) {
        DEBUG_PRINTLN("[GNSS] Quality window start");

        const uint32_t windowStart = millis();

        while (millis() - windowStart < EXTRA_QUALITY_WINDOW_MS) {
          // Keep parsing during window
          while (GNSS_PORT.available()) {
            char c = GNSS_PORT.read();
            gnss.encode(c);
          }

          // Only react to another GGA (hdop) update
          if (gnss.hdop.isUpdated() && isGoodFixForConsecutive()) {
            uint16_t h = gnss.hdop.value();
            if (!haveBestGood || h < bestHdop) {
              haveBestGood = true;
              bestLat = gnss.location.lat();
              bestLng = gnss.location.lng();
              bestSats = gnss.satellites.value();
              bestHdop = h;

              tm.Hour = gnss.time.hour();
              tm.Minute = gnss.time.minute();
              tm.Second = gnss.time.second();
              tm.Day = gnss.date.day();
              tm.Month = gnss.date.month();
              tm.Year = gnss.date.year() - 1970;
              bestEpoch = makeTime(tm);
            }
            // Early exit if excellent HDOP seen
            if (h <= HDOP_EARLY_EXIT_MAX) {
              DEBUG_PRINT("[GNSS] Quality window end (ms)=");
              DEBUG_PRINTLN(millis() - windowStart);
              break;
            }
          }

          ISBDCallback();
        }

        // Prefer the best “good” from the window; otherwise fall back to current good
        if (haveBestGood) {
          latitude = bestLat;
          longitude = bestLng;
          satellites = bestSats;
          hdop = bestHdop;
          gnssEpoch = bestEpoch;
        } else {
          latitude = gnss.location.lat();
          longitude = gnss.location.lng();
          satellites = gnss.satellites.isValid() ? gnss.satellites.value() : 0;
          hdop = gnss.hdop.isValid() ? gnss.hdop.value() : 9999;

          tm.Hour = gnss.time.hour();
          tm.Minute = gnss.time.minute();
          tm.Second = gnss.time.second();
          tm.Day = gnss.date.day();
          tm.Month = gnss.date.month();
          tm.Year = gnss.date.year() - 1970;
          gnssEpoch = makeTime(tm);
        }

        // RTC sync if GNSS is newer
        rtcEpoch = rtc.getEpoch();
        rtcDrift = rtcEpoch - gnssEpoch;

        if ((gnssEpoch > unixtime) || firstTimeFlag) {
          rtc.setEpoch(gnssEpoch);
          DEBUG_PRINT("[GNSS] Info: RTC synced ");
          printDateTime();
        } else {
          DEBUG_PRINT("[GNSS] Warning: RTC not synced! GNSS time not accurate! ");
          printDateTime();
        }

        // Store GNSS data in moSbdMessage
        moSbdMessage.latitude = (int32_t)lround(latitude * 1000000.0);
        moSbdMessage.longitude = (int32_t)lround(longitude * 1000000.0);
        moSbdMessage.satellites = satellites;
        moSbdMessage.hdop = (uint16_t)hdop;  // Keep TinyGPS++ raw units

        DEBUG_PRINT("[GNSS] Info: RTC drift = ");
        DEBUG_PRINT(rtcDrift);
        DEBUG_PRINTLN(" seconds.");
        blinkLed(5, 100);

        fixFound = true;
        break;
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

  if (!fixFound) {
    // Didn’t reach “good” fix threshold within the window
    if (haveFallback) {
      DEBUG_PRINTLN("[GNSS] Info: Using best valid fix within window (did not reach 10 good).");

      // Sync RTC from fallback if newer
      if ((fbEpoch > unixtime) || firstTimeFlag) {
        rtc.setEpoch(fbEpoch);
        DEBUG_PRINT("[GNSS] Info: RTC synced (fallback) ");
        printDateTime();
      }

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
  }

  // Close GNSS Serial port
  GNSS_PORT.end();

  // Disable power to GNSS
  disableGnssPower();

  // Record elapsed execution time
  timer.readGnss = millis() - startTime;
}
