// Configure SparkFun GPS Breakout SAM-M8Q
void configureGnss()
{
  if (gnss.begin())
  {
    gnss.setI2COutput(COM_TYPE_UBX); // Set I2C port to output UBX only (turn off NMEA noise)
    gnss.saveConfiguration();        // Save current settings to Flash and BBR
    online.gnss = true;
  }
  else
  {
    DEBUG_PRINTLN("Warning: u-blox GNSS not detected at default I2C address. Please check wiring.");
    online.gnss = false;
    //while (1);
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

    //setLedColour(pink); // Change LED colour to indicate signal acquisition

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
        int rtcDrift = rtc.getEpoch() - gnssEpoch; // Calculate time difference

        // Write data to union
        moMessage.rtcDrift = rtcDrift;
        DEBUG_PRINT("RTC drift: "); DEBUG_PRINTLN(rtcDrift);

        // Sync RTC date and time
        rtc.setTime(gnss.getHour(), gnss.getMinute(), gnss.getSecond(), gnss.getMillisecond() / 10,
                    gnss.getDay(), gnss.getMonth(), gnss.getYear() - 2000);

        rtcSyncFlag = true;
        //setLedColour(green);
        DEBUG_PRINT("RTC time synced: "); printDateTime();
      }
      ISBDCallback();
    }
    if (!rtcSyncFlag)
    {
      DEBUG_PRINTLN("Warning: RTC sync failed!");
    }
  }
  else {
    DEBUG_PRINTLN("Warning: u-blox GNSS not detected at default I2C address. Please check wiring.");
  }

  timer.sync = millis() - loopStartTime;
}

// Read the GNSS receiver
void readGnss()
{
  // Start loop timer
  unsigned long loopStartTime = millis();

  // Check if u-blox GNSS receiver is online
  if (online.gnss)
  {
    // Look for GNSS signal for up to 5 minutes
    DEBUG_PRINTLN("Beginning to listen for GNSS traffic...");
    while ((gnssFixCounter != gnssFixCounterMax) && millis() - loopStartTime < gnssTimeout * 1000UL)
    {
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

          // Get RTC's UNIX Epoch time
          time_t rtcEpoch = rtc.getEpoch();

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

          // Calculate RTC drift
          int rtcDrift = rtcEpoch - gnssEpoch;

          // Set RTC date and time
          rtc.setEpoch(gnssEpoch);

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

  }
  else
  {
    DEBUG_PRINTLN("Warning: u-blox GNSS offline!");
    return;
  }
  // Stop the loop timer
  timer.gnss = millis() - loopStartTime;
}
