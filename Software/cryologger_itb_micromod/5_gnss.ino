// Configure SparkFun GPS Breakout SAM-M8Q
void configureGnss() {

  if (gps.begin()) {
    gps.setI2COutput(COM_TYPE_UBX); // Set I2C port to output UBX only (turn off NMEA noise)
    gps.saveConfiguration();        // Save current settings to Flash and BBR
    online.gnss = true;
  }
  else {
    DEBUG_PRINTLN("Warning: SAM-M8Q not detected at default I2C address. Please check wiring.");
    online.gnss = false;
    //while (1);
  }
}

void syncRtc() {

  unsigned long loopStartTime = millis(); // Loop timer
  bool dateValid = false;
  bool timeValid = false;
  rtcSyncFlag = false;

  // Attempt to sync RTC with GNSS for up to 5 minutes
  DEBUG_PRINTLN("Attempting to sync RTC with GNSS...");

  while ((!dateValid || !timeValid) && millis() - loopStartTime < 1UL * 60UL * 1000UL) {

    dateValid = gps.getDateValid();
    timeValid = gps.getTimeValid();

    // Sync RTC with GNSS if date and time are valid
    if (dateValid && timeValid) {
      rtc.setTime(gps.getHour(), gps.getMinute(), gps.getSecond(), gps.getMillisecond() / 10,
                  gps.getDay(), gps.getMonth(), gps.getYear() - 2000);

      rtcSyncFlag = true; // Set flag
      blinkLed(10, 50);
      DEBUG_PRINT("RTC time synced: "); printDateTime();
    }

    //blinkLed(1, 500);
    ISBDCallback();
  }
  if (!rtcSyncFlag) {
    DEBUG_PRINTLN("Warning: RTC sync failed!");
  }
}

// Read the GPS Breakout SAM-M8Q
void readGnss() {

  if (online.gnss) {
    unsigned long loopStartTime = millis(); // Loop timer

    // Begin listening to the GNSS
    DEBUG_PRINTLN("Beginning to listen for GNSS traffic...");

    // Look for GNSS signal for up to 5 minutes
    while ((gnssFixCounter != gnssFixCounterMax) && millis() - loopStartTime < 1UL * 60UL * 1000UL) {

#if DEBUG
      char gnssBuffer[75];
      sprintf(gnssBuffer, "%04u-%02d-%02d %02d:%02d:%02d,%ld,%ld,%d,%d,%d",
              gps.getYear(), gps.getMonth(), gps.getDay(),
              gps.getHour(), gps.getMinute(), gps.getSecond(),
              gps.getLatitude(), gps.getLongitude(), gps.getSIV(),
              gps.getFixType(), gps.getPDOP());
      //DEBUG_PRINTLN(gnssBuffer);
#endif

      // Check for GNSS fix
      if (gps.getFixType() > 0) {
        gnssFixCounter += 1; // Increment counter
      }

      // Check if enough valid GNSS fixes have been collected
      if ((gps.getFixType() > 0) && (gnssFixCounter == gnssFixCounterMax)) {

        DEBUG_PRINTLN("A GNSS fix was found!");

        // Record GNSS coordinates
        long latitude = gps.getLatitude();
        long longitude = gps.getLongitude();
        byte satellites = gps.getSIV();
        byte fix = gps.getFixType();
        unsigned int pdop = gps.getPDOP();

        // Write data to union
        moMessage.latitude = latitude;
        moMessage.longitude = longitude;
        moMessage.satellites = satellites;
        moMessage.pdop = pdop;
        break;

        // Sync RTC with GNSS if date and time are valid
        if (gps.getDateValid() && gps.getTimeValid()) {
          rtc.setTime(gps.getHour(), gps.getMinute(), gps.getSecond(), gps.getMillisecond() / 10,
                      gps.getDay(), gps.getMonth(), gps.getYear() - 2000);
          DEBUG_PRINT("RTC time synced: "); printDateTime();
        }

      }
      ISBDCallback();
    }

    // Check if a GNSS fix was acquired
    if (gnssFixCounter < gnssFixCounterMax) {
      DEBUG_PRINTLN("Warning: No GNSS fix was found!");
    }

    // Reset valFix counter
    gnssFixCounter = 0;

    unsigned long loopEndTime = millis() - loopStartTime;
    DEBUG_PRINT("readGnss() function execution: ");
    DEBUG_PRINT(loopEndTime);
    DEBUG_PRINTLN(" ms");
  }
  else {
    DEBUG_PRINTLN("Warning: GNSS offline!");
    return;
  }
}
