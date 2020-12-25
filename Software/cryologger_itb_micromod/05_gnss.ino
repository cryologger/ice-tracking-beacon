// Configure SparkFun GPS Breakout SAM-M8Q
void configureGnss() {

  if (gnss.begin()) {
    gnss.setI2COutput(COM_TYPE_UBX); // Set I2C port to output UBX only (turn off NMEA noise)
    gnss.saveConfiguration();        // Save current settings to Flash and BBR
    online.gnss = true;
  }
  else {
    DEBUG_PRINTLN("Warning: GNSS not detected at default I2C address. Please check wiring.");
    online.gnss = false;
    //while (1);
  }
}

void syncRtc() {

  // Check if GNSS receiver is online
  if (online.gnss) {
    unsigned long loopStartTime = millis(); // Loop timer
    bool dateValid = false;
    bool timeValid = false;
    rtcSyncFlag = false;

    // Attempt to sync RTC with GNSS for up to 5 minutes
    DEBUG_PRINTLN("Attempting to sync RTC with GNSS...");

    while ((!dateValid || !timeValid) && millis() - loopStartTime < 5UL * 60UL * 1000UL) {

      dateValid = gnss.getDateValid();
      timeValid = gnss.getTimeValid();

      // Sync RTC with GNSS if date and time are valid
      if (dateValid && timeValid) {
        rtc.setTime(gnss.getHour(), gnss.getMinute(), gnss.getSecond(), gnss.getMillisecond() / 10,
                    gnss.getDay(), gnss.getMonth(), gnss.getYear() - 2000);

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
  else {
    Serial.println(F("Warning: No u-blox GNSS receiver detected on Qwiic bus!"));
  }
}

// Read the GNSS receiver
void readGnss() {

  // Check if u-blox GNSS receiver is online
  if (online.gnss) {
    unsigned long loopStartTime = millis(); // Loop timer

    // Begin listening to the GNSS
    DEBUG_PRINTLN("Beginning to listen for GNSS traffic...");

    // Look for GNSS signal for up to 5 minutes
    while ((gnssFixCounter != gnssFixCounterMax) && millis() - loopStartTime < 5UL * 60UL * 1000UL) {

#if DEBUG_GNSS
      char gnssBuffer[75];
      sprintf(gnssBuffer, "%04u-%02d-%02d %02d:%02d:%02d,%ld,%ld,%d,%d,%d",
              gnss.getYear(), gnss.getMonth(), gnss.getDay(),
              gnss.getHour(), gnss.getMinute(), gnss.getSecond(),
              gnss.getLatitude(), gnss.getLongitude(), gnss.getSIV(),
              gnss.getFixType(), gnss.getPDOP());
      DEBUG_PRINTLN(gnssBuffer);
#endif

      // Check for GNSS fix
      if (gnss.getFixType() > 0) {
        gnssFixCounter += 1; // Increment GNSS fix counter
      }

      // Check if enough valid GNSS fixes have been collected
      if ((gnss.getFixType() > 0) && (gnssFixCounter == gnssFixCounterMax)) {

        DEBUG_PRINTLN("A GNSS fix was found!");

        // Write data to union
        moMessage.latitude = gnss.getLatitude();
        moMessage.longitude = gnss.getLongitude();
        moMessage.satellites = gnss.getSIV();
        moMessage.pdop = gnss.getPDOP();

        // Sync RTC with GNSS if date and time are valid
        if (gnss.getDateValid() && gnss.getTimeValid()) {
          rtc.setTime(gnss.getHour(), gnss.getMinute(), gnss.getSecond(), gnss.getMillisecond() / 10,
                      gnss.getDay(), gnss.getMonth(), gnss.getYear() - 2000);


          // Calculate RTC drift
          tmElements_t tm;
          tm.Year = gnss.getYear() - 1970;
          tm.Month = gnss.getMonth();
          tm.Day = gnss.getDay();
          tm.Hour = gnss.getHour();
          tm.Minute = gnss.getMinute();
          tm.Second = gnss.getSecond();

          // Convert GNSS date and time to UNIX Epoch time (tmElements to time_t)
          time_t gnssEpoch = makeTime(tm);

          // Get RTC's UNIX Epoch time
          time_t rtcEpoch = rtc.getEpoch();

          // Calculate RTC drift
          int rtcDrift = rtcEpoch - gnssEpoch;

          // Write data to union
          moMessage.rtcDrift = rtcDrift;

          DEBUG_PRINT("RTC drift: "); DEBUG_PRINTLN(rtcDrift);
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
