// Configure SparkFun GPS Breakout SAM-M8Q
void configureGnss()
{
  if (gnss.begin())
  {
    gnss.setNavigationFrequency(1);
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

// Read the GNSS receiver
void readGnss()
{
  // Start loop timer
  unsigned long loopStartTime = millis();

  bool gnssFixFlag = false; // Reset fix flag
  byte gnssFixCounter = 0; // Reset fix counter
  bool rtcSyncFlag = false; // Clear RTC sync flag

  // Check if GNSS receiver is online
  if (online.gnss)
  {
    // Acquire valid GNSS position fix
    Serial.println(F("Acquiring GNSS fix..."));

    while (!gnssFixFlag && millis() - loopStartTime < gnssTimeout * 1000UL)
    {
      byte fixType = gnss.getFixType();
      bool timeValidFlag = gnss.getTimeValid();
      bool dateValidFlag = gnss.getDateValid();

#if DEBUG_GNSS
      char gnssBuffer[75];
      sprintf(gnssBuffer, "%04u-%02d-%02d %02d:%02d:%02d,%ld,%ld,%d,%d,%d,%d,%d",
              gnss.getYear(), gnss.getMonth(), gnss.getDay(),
              gnss.getHour(), gnss.getMinute(), gnss.getSecond(),
              gnss.getLatitude(), gnss.getLongitude(), gnss.getSIV(),
              gnss.getPDOP(), fixType, timeValidFlag, dateValidFlag);
      DEBUG_PRINTLN(gnssBuffer);
#endif

      // Check for 3D GNSS fix
      if (fixType == 3)
      {
        gnssFixCounter += 2; // Increment counter
      }
      else if (fixType == 2)
      {
        gnssFixCounter += 1; // Increment counter
      }

      // Check if GNSS fix threshold has been reached
      if (gnssFixCounter >= gnssFixCounterMax) {

        DEBUG_PRINTLN("A GNSS fix was found!");
        gnssFixFlag = true; // Set fix flag

        // Write data to union
        moMessage.latitude = gnss.getLatitude();
        moMessage.longitude = gnss.getLongitude();
        moMessage.satellites = gnss.getSIV();
        moMessage.pdop = gnss.getPDOP();

        // Sync RTC with GNSS
        if ((fixType == 3) && timeValidFlag && dateValidFlag) {

          // Calculate RTC drift
          tmElements_t tm;
          tm.Year = gnss.getYear() - 1970;
          tm.Month = gnss.getMonth();
          tm.Day = gnss.getDay();
          tm.Hour = gnss.getHour();
          tm.Minute = gnss.getMinute();
          tm.Second = gnss.getSecond();
          time_t gnssEpoch = makeTime(tm); // Convert tmElements to time_t

          // Get the RTC's date and time
          rtc.getTime();

          // Calculate drift
          int rtcDrift = rtc.getEpoch() - gnssEpoch;

          DEBUG_PRINT("RTC drift: "); DEBUG_PRINTLN(rtcDrift);

          // Write data to union
          moMessage.rtcDrift = rtcDrift;

          // Sync RTC date and time
          rtc.setTime(gnss.getHour(), gnss.getMinute(), gnss.getSecond(), gnss.getMillisecond() / 10,
                      gnss.getDay(), gnss.getMonth(), gnss.getYear() - 2000);

          rtcSyncFlag = true;
          DEBUG_PRINT("RTC time synced: "); printDateTime();
        }
        else
        {
          DEBUG_PRINTLN("Warning: RTC not synced due to invalid GNSS fix!");
        }
      }
      ISBDCallback();
    }

    // Check if a GNSS fix was acquired
    if (!gnssFixFlag)
    {
      DEBUG_PRINTLN("Warning: No GNSS fix was found!");
    }
  }
  else
  {
    DEBUG_PRINTLN("Warning: u-blox GNSS offline!");
  }

  // Stop the loop timer
  timer.gnss = millis() - loopStartTime;
}

// Synchornize the RTC with the GNSS
void syncRtc()
{
  // Start loop timer
  unsigned long loopStartTime = millis();

  bool rtcSyncFlag = false;       // Clear RTC sync flag
  bool timeValidityFlag = false;  // Clear time and date validity flag
  byte timeValidityCounter = 0;   // Reset time validity counter

  DEBUG_PRINTLN("Attempting to sync RTC with GNSS...");
  
  // Sync RTC with GNSS
  while (!timeValidityFlag && millis() - loopStartTime < rtcSyncTimeout * 1000UL)
  {
    byte fixType = gnss.getFixType();
    bool timeValidFlag = gnss.getTimeValid();
    bool dateValidFlag = gnss.getDateValid();

    // Check for GNSS fix and valid time and date flags
    if ((fixType == 3) && timeValidFlag && dateValidFlag)
    {
      timeValidityCounter += 2; // Increment GNSS fix counter
    }

    if ((fixType == 2) && timeValidFlag && dateValidFlag)
    {
      timeValidityCounter += 1; // Increment GNSS fix counter
    }
    
    // Sync RTC with GNSS if date and time are valid
    if (timeValidityCounter >= 10)
    {
      timeValidityFlag = true; // Set flag
      
      // Convert to Unix Epoch time
      tmElements_t tm;
      tm.Year = gnss.getYear() - 1970;
      tm.Month = gnss.getMonth();
      tm.Day = gnss.getDay();
      tm.Hour = gnss.getHour();
      tm.Minute = gnss.getMinute();
      tm.Second = gnss.getSecond();
      time_t gnssEpoch = makeTime(tm);
      rtc.getTime(); // Get the RTC's date and time

      // Calculate drift
      int rtcDrift = rtc.getEpoch() - gnssEpoch; 

      DEBUG_PRINT("RTC drift: "); DEBUG_PRINTLN(rtcDrift);

      // Write data to union
      moMessage.rtcDrift = rtcDrift;

      // Set RTC date and time
      rtc.setTime(gnss.getHour(), gnss.getMinute(), gnss.getSecond(), gnss.getMillisecond() / 10,
                  gnss.getDay(), gnss.getMonth(), gnss.getYear() - 2000);

      rtcSyncFlag = true; // Set flag
      DEBUG_PRINT("RTC time synced to: "); printDateTime();
    }
    ISBDCallback();
  }
  if (!rtcSyncFlag)
  {
    DEBUG_PRINTLN("Warning: RTC sync failed!");
  }
  // Stop loop timer
  timer.sync = millis() - loopStartTime;
}
