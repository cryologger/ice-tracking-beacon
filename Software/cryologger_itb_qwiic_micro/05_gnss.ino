// Configure GNSS receiver
void configureGnss() 
{
  if (gnss.begin()) {
    gnss.setI2COutput(COM_TYPE_UBX); // Set I2C port to output UBX only (turn off NMEA noise)
    //gnss.saveConfiguration(); // Save current settings to Flash and BBR
    online.gnss = true;
  }
  else {
    DEBUG_PRINTLN("Warning: GNSS receiver not detected at default I2C address. Please check wiring.");
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

    while ((!dateValid || !timeValid) && millis() - loopStartTime < gnssDelay * 1000UL) {

      dateValid = gnss.getDateValid();
      timeValid = gnss.getTimeValid();

      // Sync RTC with GNSS if date and time are valid
      if (dateValid && timeValid) {

        // Calculate RTC drift
        tmElements_t tm;
        tm.Year = gnss.getYear() - 1970;
        tm.Month = gnss.getMonth();
        tm.Day = gnss.getDay();
        tm.Hour = gnss.getHour();
        tm.Minute = gnss.getMinute();
        tm.Second = gnss.getSecond();
        time_t gnssEpoch = makeTime(tm); // Convert tmElements to time_t
        rtc.updateTime(); // Update time variables from RTC
        int rtcDrift = rtc.getEpoch() - gnssEpoch; // Calculate time difference

        // Write data to union
        moMessage.rtcDrift = rtcDrift;
        DEBUG_PRINT("RTC drift: "); DEBUG_PRINTLN(rtcDrift);

        // Sync RTC date and time
        //rtc.setTime(gnss.getHour(), gnss.getMinute(), gnss.getSecond());
        //rtc.setDate(gnss.getDay(), gnss.getMonth(), gnss.getYear() - 2000);

        // Sync RTC date and time (RV8803)
        rtc.setTime(gnss.getSecond(), gnss.getMinute(), gnss.getHour(), 0,
                    gnss.getDay(), gnss.getMonth(), gnss.getYear());

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

    while ((gnssFixCounter != gnssFixCounterMax) && millis() - loopStartTime < gnssDelay * 1000UL) {

#if DEBUG_GNSS
      char gnssBuffer[100];
      sprintf(gnssBuffer, "%04u-%02d-%02d %02d:%02d:%02d,%ld,%ld,%d,%d,%d",
              gnss.getYear(), gnss.getMonth(), gnss.getDay(),
              gnss.getHour(), gnss.getMinute(), gnss.getSecond(),
              gnss.getLatitude(), gnss.getLongitude(), gnss.getSIV(),
              gnss.getFixType(), gnss.getPDOP());
      DEBUG_PRINTLN(gnssBuffer);
#endif

      // Check for GNSS fix
      if (gnss.getFixType() > 0) {
        gnssFixCounter += 1; // Increment counter
      }

      // Check if enough valid GNSS fixes have been collected
      if ((gnss.getFixType() > 0) && (gnssFixCounter == gnssFixCounterMax)) {

        DEBUG_PRINTLN("A GNSS fix was found!");

        bool dateValid = gnss.getDateValid();
        bool timeValid = gnss.getTimeValid();

        // Write data to union
        moMessage.latitude = gnss.getLatitude();
        moMessage.longitude = gnss.getLongitude();
        moMessage.satellites = gnss.getSIV();
        moMessage.pdop = gnss.getPDOP();

        // Sync RTC with GNSS if date and time are valid
        if (dateValid && timeValid) {

          // Calculate RTC drift
          tmElements_t tm;
          tm.Year = gnss.getYear() - 1970;
          tm.Month = gnss.getMonth();
          tm.Day = gnss.getDay();
          tm.Hour = gnss.getHour();
          tm.Minute = gnss.getMinute();
          tm.Second = gnss.getSecond();
          time_t gnssEpoch = makeTime(tm); // Convert tmElements to time_t
          rtc.updateTime(); // Update time variables from RTC
          int rtcDrift = rtc.getEpoch() - gnssEpoch; // Calculate time difference

          // Write data to union
          moMessage.rtcDrift = rtcDrift;
          DEBUG_PRINT("RTC drift: "); DEBUG_PRINTLN(rtcDrift);

          // Sync RTC date and time
          //rtc.setTime(gnss.getHour(), gnss.getMinute(), gnss.getSecond());
          //rtc.setDate(gnss.getDay(), gnss.getMonth(), gnss.getYear() - 2000);

          // Sync RTC date and time (RV8803)
          rtc.setTime(gnss.getSecond(), gnss.getMinute(), gnss.getHour(), 0,
                      gnss.getDay(), gnss.getMonth(), gnss.getYear());

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
