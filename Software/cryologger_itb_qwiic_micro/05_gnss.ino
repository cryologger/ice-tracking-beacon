// Configure SparkFun GPS Breakout SAM-M8Q
void configureGnss() {
  blinkLed(1, 1000);
  if (gps.begin()) {
    gps.setI2COutput(COM_TYPE_UBX); // Set I2C port to output UBX only (turn off NMEA noise)
    //gps.saveConfiguration(); // Save current settings to Flash and BBR
    online.gnss = true;
  }
  else {
    DEBUG_PRINTLN("Warning: SAM-M8Q not detected at default I2C address. Please check wiring.");
    online.gnss = false;
  }
}

// Synchronize RTC with GNSS
void syncRtc() {

  // Start loop timer
  unsigned long loopStartTime = millis();

  if (online.gnss) {
    bool dateValid = false;
    bool timeValid = false;
    rtcSyncFlag = false;

    // Attempt to sync RTC with GNSS for up to 5 minutes
    DEBUG_PRINTLN("Attempting to sync RTC with GNSS...");

    setLedColour(pink);

    while ((!dateValid || !timeValid) && millis() - loopStartTime < 1UL * 10UL * 1000UL) {

      dateValid = gps.getDateValid();
      timeValid = gps.getTimeValid();

      // Sync RTC with GNSS if date and time are valid
      if (dateValid && timeValid) {

        // Calculate RTC drift
        tmElements_t tm;
        tm.Year = gps.getYear() - 1970;
        tm.Month = gps.getMonth();
        tm.Day = gps.getDay();
        tm.Hour = gps.getHour();
        tm.Minute = gps.getMinute();
        tm.Second = gps.getSecond();
        time_t gnssEpoch = makeTime(tm); // Convert tmElements to time_t
        rtc.updateTime(); // Update time variables from RTC
        int rtcDrift = rtc.getEpoch() - gnssEpoch; // Calculate time difference

        // Write data to union
        moMessage.rtcDrift = rtcDrift;
        DEBUG_PRINT("RTC drift: "); DEBUG_PRINTLN(rtcDrift);

        // Sync RTC date and time
        //rtc.setTime(gps.getHour(), gps.getMinute(), gps.getSecond());
        //rtc.setDate(gps.getDay(), gps.getMonth(), gps.getYear() - 2000);

        // Sync RTC date and time (RV8803)
        rtc.setTime(gps.getSecond(), gps.getMinute(), gps.getHour(), 0,
                    gps.getDay(), gps.getMonth(), gps.getYear());

        rtcSyncFlag = true;
        setLedColour(green);
        DEBUG_PRINT("RTC time synced: "); printDateTime();
      }

      ISBDCallback();
    }
    if (!rtcSyncFlag) {
      DEBUG_PRINTLN("Warning: RTC sync failed!");
      setLedColour(red);
    }
  }
}

// Read GNSS
void readGnss() {

  if (online.gnss) {

    unsigned long loopStartTime = millis(); // Start loop timer
    gnssFixCounter = 0; // Reset fix counter

    // Look for GNSS signal for up to 5 minutes
    DEBUG_PRINTLN("Beginning to listen for GNSS traffic...");

    // Set LED
    setLedColour(cyan);

    while ((gnssFixCounter != gnssFixCounterMax) && millis() - loopStartTime < 1UL * 10UL * 1000UL) {

#if DEBUG_GNSS
      char gnssBuffer[100];
      sprintf(gnssBuffer, "%04u-%02d-%02d %02d:%02d:%02d,%ld,%ld,%d,%d,%d",
              gps.getYear(), gps.getMonth(), gps.getDay(),
              gps.getHour(), gps.getMinute(), gps.getSecond(),
              gps.getLatitude(), gps.getLongitude(), gps.getSIV(),
              gps.getFixType(), gps.getPDOP());
      DEBUG_PRINTLN(gnssBuffer);
#endif

      // Check for GNSS fix
      if (gps.getFixType() > 0) {
        gnssFixCounter += 1; // Increment counter
      }

      // Check if enough valid GNSS fixes have been collected
      if ((gps.getFixType() > 0) && (gnssFixCounter == gnssFixCounterMax)) {

        DEBUG_PRINTLN("A GNSS fix was found!");

        long latitude = gps.getLatitude();
        long longitude = gps.getLongitude();
        byte satellites = gps.getSIV();
        byte fix = gps.getFixType();
        unsigned int pdop = gps.getPDOP();
        bool dateValid = gps.getDateValid();
        bool timeValid = gps.getTimeValid();

        // Write data to union
        moMessage.latitude = latitude;
        moMessage.longitude = longitude;
        moMessage.satellites = satellites;
        moMessage.pdop = pdop;

        // Sync RTC with GNSS if date and time are valid
        if (dateValid && timeValid) {

          // Calculate RTC drift
          tmElements_t tm;
          tm.Year = gps.getYear() - 1970;
          tm.Month = gps.getMonth();
          tm.Day = gps.getDay();
          tm.Hour = gps.getHour();
          tm.Minute = gps.getMinute();
          tm.Second = gps.getSecond();
          time_t gnssEpoch = makeTime(tm); // Convert tmElements to time_t
          rtc.updateTime(); // Update time variables from RTC
          int rtcDrift = rtc.getEpoch() - gnssEpoch; // Calculate time difference

          // Write data to union
          moMessage.rtcDrift = rtcDrift;
          DEBUG_PRINT("RTC drift: "); DEBUG_PRINTLN(rtcDrift);

          // Sync RTC date and time
          //rtc.setTime(gps.getHour(), gps.getMinute(), gps.getSecond());
          //rtc.setDate(gps.getDay(), gps.getMonth(), gps.getYear() - 2000);

          // Sync RTC date and time (RV8803)
          rtc.setTime(gps.getSecond(), gps.getMinute(), gps.getHour(), 0,
                      gps.getDay(), gps.getMonth(), gps.getYear());

          DEBUG_PRINT("RTC time synced: "); printDateTime();
        }
        setLedColour(green);
      }
      ISBDCallback();
    }

    // Check if a GNSS fix was acquired
    if (gnssFixCounter < gnssFixCounterMax) {
      DEBUG_PRINTLN("Warning: No GNSS fix was found!");
      setLedColour(red);
    }

    unsigned long loopEndTime = millis() - loopStartTime; // Stop loop timer
    DEBUG_PRINT("readGnss() function execution: "); DEBUG_PRINT(loopEndTime); DEBUG_PRINTLN(" ms");
  }
}
