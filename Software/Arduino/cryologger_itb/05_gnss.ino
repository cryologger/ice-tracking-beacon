/*
  GNSS Module

  This module handles reading GNSS data from an attached receiver,
  parsing NMEA sentences via TinyGPS++. It attempts to acquire a valid
  fix within a user-defined timeout, synchronizes the RTC if the GNSS
  time is newer, and updates the moSbdMessage structure with location
  and satellite data.
*/

// ----------------------------------------------------------------------------
// Read the GNSS receiver.
// If a valid fix is found, sync the RTC if newer than the current unixtime,
// update moSbdMessage with GNSS data, and log RTC drift.
// ----------------------------------------------------------------------------
void readGnss() {
  // Start execution timer
  unsigned long startTime = millis();

  // Clear flags.
  bool fixFound = false;
  bool charsSeen = false;

  // Reset GNSS fix counter
  byte fixCounter = 0;

  // Enable power to GNSS
  enableGnssPower();

  DEBUG_PRINTLN("[GNSS] Info: Beginning to listen for GNSS traffic...");

  // Initialize GNSS serial port
  GNSS_PORT.begin(9600);
  myDelay(1000);

  // Configure GNSS update/output rates
  GNSS_PORT.println("$PMTK220,1000*1F");  // Set NMEA update rate to 1 Hz
  myDelay(100);

  // Set NMEA sentence output frequencies to GGA and RMC
  GNSS_PORT.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");
  myDelay(100);

  // Optional antenna update configuration
  // GNSS_PORT.println("$PGCMD,33,1*6C"); // Enable antenna updates
  // GNSS_PORT.println("$PGCMD,33,0*6D"); // Disable antenna updates

  // Look for GNSS signal up to gnssTimeout seconds
  while (!fixFound && (millis() - startTime < gnssTimeout * 1000UL)) {
    if (GNSS_PORT.available()) {
      charsSeen = true;
      char c = GNSS_PORT.read();
#if DEBUG_GNSS
      Serial.write(c);  // Echo NMEA to Serial for debugging
#endif
      if (gnss.encode(c)) {
        // Valid fix if fix quality > 0, not stale, and satellites > 0.
        if (isValidGnssFix()) {

          DEBUG_PRINT(" Pass");  // Debugging only
          fixCounter++;          // Increment fix counter

          // Wait until enough consecutive fixes have been collected
          if (fixCounter >= 10) {
            fixFound = true;

            // Convert GNSS date/time to epoch
            tm.Hour = gnss.time.hour();
            tm.Minute = gnss.time.minute();
            tm.Second = gnss.time.second();
            tm.Day = gnss.date.day();
            tm.Month = gnss.date.month();
            tm.Year = gnss.date.year() - 1970;  // Offset from 1970

            gnssEpoch = makeTime(tm);         // Change the tm structure into time_t (seconds since epoch)
            rtcEpoch = rtc.getEpoch();        // Get RTC epoch time
            rtcDrift = rtcEpoch - gnssEpoch;  // Calculate RTC drift

            DEBUG_PRINTLN();
            DEBUG_PRINT("[GNSS] Info: gnssEpoch = ");
            DEBUG_PRINTLN(gnssEpoch);
            DEBUG_PRINT("[GNSS] Info: rtcEpoch  = ");
            DEBUG_PRINTLN(rtcEpoch);

            // Sync RTC if gnssEpoch is newer
            if ((gnssEpoch > unixtime) || firstTimeFlag) {
              rtc.setEpoch(gnssEpoch);
              DEBUG_PRINT("[GNSS] Info: RTC synced ");
              printDateTime();
            } else {
              DEBUG_PRINT("[GNSS] Warning: RTC not synced! GNSS time not accurate! ");
              printDateTime();
            }

            // Store GNSS data in moSbdMessage
            moSbdMessage.latitude = gnss.location.lat() * 1000000;
            moSbdMessage.longitude = gnss.location.lng() * 1000000;
            moSbdMessage.satellites = gnss.satellites.value();
            moSbdMessage.hdop = gnss.hdop.value();

            DEBUG_PRINT("[GNSS] Info: RTC drift = ");
            DEBUG_PRINT(rtcDrift);
            DEBUG_PRINTLN(" seconds.");
            blinkLed(5, 100);
          }
        } else {
          DEBUG_PRINT(" Fail");  // Debugging only
        }
      }
    }

    // Call callback during acquisition
    ISBDCallback();

    // If no data after ~5 seconds, break
    if ((millis() - startTime) > 5000 && gnss.charsProcessed() < 10) {
      DEBUG_PRINTLN("[GNSS] Warning: No GNSS data received. Please check wiring.");
      break;
    }
  }

  if (!fixFound) {
    DEBUG_PRINTLN("[GNSS] Warning: No GNSS fix found!");
  }

  // Close GNSS Serial port
  GNSS_PORT.end();

  // Disable power to GNSS
  disableGnssPower();

  // Record elapsed execution time
  timer.readGnss = millis() - startTime;
}

// ----------------------------------------------------------------------------
// Check for valid GNSS fix.
// ----------------------------------------------------------------------------
bool isValidGnssFix() {
  return (
    gnssFix.value()
    && atoi(gnssFix.value()) > 0
    && gnssFix.age() < 1000
    && gnssValidity.value()
    && strcmp(gnssValidity.value(), "A") == 0
    && gnssValidity.age() < 1000
    && gnss.satellites.isValid() && gnss.satellites.value() > 0);
}
