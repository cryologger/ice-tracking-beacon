// Configure GNSS receiver
void configureGnss()
{
  if (!gnss.begin())
  {
    DEBUG_PRINTLN("Warning: GNSS receiver not detected at default I2C address. Please check wiring.");
    online.gnss = false;
    setLedColour(red);
    //while (true); // Force Watchdog Timer to reset the system *DEBUGGING ONLY*
  }
  else
  {
    gnss.setI2COutput(COM_TYPE_UBX); // Set I2C port to output UBX only (turn off NMEA noise)
    //gnss.saveConfiguration(); // Save current settings to Flash and BBR
    online.gnss = true;
  }
}

// Synchronize RTC with GNSS
void syncRtc()
{
  // Start loop timer
  unsigned long loopStartTime = millis();

  // Check if GNSS initialized successfully
  if (online.gnss)
  {
    // Clear flags
    bool dateValid = false;
    bool timeValid = false;
    rtcSyncFlag = false;

    // Attempt to sync RTC with GNSS for up to 5 minutes
    DEBUG_PRINTLN("Attempting to sync RTC with GNSS...");

    setLedColour(pink); // Change LED colour to indicate signal acquisition

    while ((!dateValid || !timeValid) && millis() - loopStartTime < gnssTimeout * 1000UL)
    {
      dateValid = gnss.getDateValid();
      timeValid = gnss.getTimeValid();

      // Sync RTC with GNSS if date and time are valid
      if (dateValid && timeValid)
      {
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
        rtc.setTime(gnss.getSecond(), gnss.getMinute(), gnss.getHour(), 0,
                    gnss.getDay(), gnss.getMonth(), gnss.getYear());

        rtcSyncFlag = true;
        setLedColour(green);
        DEBUG_PRINT("RTC time synced: "); printDateTime();
      }
      ISBDCallback();
    }
    if (!rtcSyncFlag)
    {
      DEBUG_PRINTLN("Warning: RTC sync failed!");
      setLedColour(red); // Change LED colour to indicate RTC sync failure
    }
  }
}

// Read GNSS
void readGnss() {

  // Check if GNSS initialized successfully
  if (online.gnss)
  {
    setLedColour(cyan); // Change LED colour to indicate signal acquisition

    unsigned long loopStartTime = millis(); // Start loop timer
    bool gnssFixFlag = false; // GNSS valid fix flag
    byte gnssFixCounter = 0; // GNSS valid fix counter

    // Look for GNSS signal for up to 5 minutes
    DEBUG_PRINTLN("Beginning to listen for GNSS traffic...");

    while (!gnssFixFlag && millis() - loopStartTime < gnssTimeout * 1000UL)
    {
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
      if (gnss.getFixType() > 0)
      {
        gnssFixCounter += 1; // Increment counter
      }

      // Check if enough valid GNSS fixes have been collected
      if ((gnss.getFixType() == 3) || (gnssFixCounter == gnssFixCounterMax))
      {
        // Set GNSS valid fix flag
        gnssFixFlag = true;

        DEBUG_PRINTLN("A GNSS fix was found!");

        // Write data to union
        moMessage.latitude = gnss.getLatitude();
        moMessage.longitude = gnss.getLongitude();
        moMessage.satellites = gnss.getSIV();
        moMessage.pdop = gnss.getPDOP();

        // Sync RTC with GNSS if date and time are valid
        if (gnss.getDateValid() && gnss.getTimeValid())
        {
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

          // Sync RTC date and time with GNSS
          rtc.setTime(gnss.getSecond(), gnss.getMinute(), gnss.getHour(), 0,
                      gnss.getDay(), gnss.getMonth(), gnss.getYear());

          DEBUG_PRINT("RTC time synced: ");
          printDateTime();
        }
        setLedColour(green); // Change LED colour to indicate GNSS fix was found
      }
      // Call callback
      ISBDCallback();
    }

    // Check if a GNSS fix was acquired
    if (!gnssFixFlag)
    {
      DEBUG_PRINTLN("Warning: No GNSS fix was found!");
      setLedColour(red); // Change LED colour to indicate no GNSS fix found
    }

    unsigned long loopEndTime = millis() - loopStartTime; // Stop loop timer
    DEBUG_PRINT("readGnss() function execution: "); DEBUG_PRINT(loopEndTime); DEBUG_PRINTLN(" ms");
  }
  else
  {
    DEBUG_PRINTLN("Warning: GNSS receiver offline!");
    setLedColour(red); // Change LED colour to indicate Iridium is offline
  }
}
