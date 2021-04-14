// Configure u-blox GNSS
void configureGnss()
{
  if (gnss.begin())
  {
    //gnss.enableDebugging();                           // Uncomment to enable GNSS debug messages on Serial
    gnss.setI2COutput(COM_TYPE_UBX);                  // Set I2C port to output UBX only
    gnss.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);  // Save communications port settings to flash and BBR
    gnss.setNavigationFrequency(1);                   // Produce two solutions per second
    gnss.setAutoPVT(true);                            // Enable automatic NAV-PVT messages
    online.gnss = true;
  }
  else
  {
    DEBUG_PRINTLN("Warning: u-blox GNSS not detected at default I2C address. Please check wiring.");
    online.gnss = false;
  }
}

// Acquire valid GNSS fix and sync RTC
void syncRtc()
{
  // Start loop timer
  unsigned long loopStartTime = millis();

  // Check if microSD and u-blox GNSS initialized successfully
  if (online.gnss)
  {
    // Clear flag
    bool rtcSyncFlag = false;
    int gnssFixCounter = 0;

    setLedColour(CRGB::Cyan);
    DEBUG_PRINTLN("Info: Attempting to synchronize RTC with GNSS...");

    // Attempt to acquire a valid GNSS position fix for up to 5 minutes
    while (!rtcSyncFlag && millis() - loopStartTime < gnssTimeout * 60UL * 1000UL)
    {
      petDog(); // Reset watchdog timer

      // Check for UBX-NAV-PVT messages
      if (gnss.getPVT())
      {
        // Blink LED
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

        bool dateValidFlag = gnss.getDateValid();
        bool timeValidFlag = gnss.getTimeValid();
        byte fixType = gnss.getFixType();

#if DEBUG_GNSS
        char gnssBuffer[100];
        sprintf(gnssBuffer, "%04u-%02d-%02d %02d:%02d:%02d.%03d,%ld,%ld,%d,%d,%d,%d,%d",
                gnss.getYear(), gnss.getMonth(), gnss.getDay(),
                gnss.getHour(), gnss.getMinute(), gnss.getSecond(), gnss.getMillisecond(),
                gnss.getLatitude(), gnss.getLongitude(), gnss.getSIV(),
                gnss.getPDOP(), fixType, dateValidFlag, timeValidFlag);
        DEBUG_PRINTLN(gnssBuffer);
#endif

        if (fixType >= 2)
        {
          gnssFixCounter++;
        }

        // Check if date and time are valid and sync RTC with GNSS
        if (gnssFixCounter > 3 && fixType == 3 && dateValidFlag && timeValidFlag)
        {
          rtc.updateTime();                               // Update time variables from RTC
          unsigned long rtcEpoch = rtc.getEpoch();        // Get RTC epoch time
          unsigned long gnssEpoch = gnss.getUnixEpoch();  // Get GNSS epoch time
          rtc.setEpoch(gnssEpoch);                        // Set RTC date and time
          unsigned long rtcDrift = gnssEpoch - rtcEpoch;  // Calculate RTC drift
          rtcSyncFlag = true;                             // Set flag

          // Write data to union
          moMessage.rtcDrift = rtcDrift;

          // Change LED colour
          setLedColour(CRGB::Green);

          DEBUG_PRINT("Info: RTC drift: "); DEBUG_PRINTLN(rtcDrift);
          DEBUG_PRINT("Info: RTC time synced to "); printDateTime();
        }
      }
    }
    if (!rtcSyncFlag)
    {
      DEBUG_PRINTLN("Warning: Unable to sync RTC!");
    }
  }
  else
  {
    DEBUG_PRINTLN("Warning: u-blox GNSS is offline!");
  }

  // Stop the loop timer
  timer.syncRtc = millis() - loopStartTime;
}


// Acquire valid GNSS fix and sync RTC
void readGnss()
{
  // Start loop timer
  unsigned long loopStartTime = millis();

  // Check if microSD and u-blox GNSS initialized successfully
  if (online.gnss)
  {
    // Clear flag
    bool gnssFixFlag = false;
    setLedColour(CRGB::Cyan);
    DEBUG_PRINTLN("Info: Attempting to acquire a GNSS fix...");

    // Attempt to acquire a valid GNSS position fix for up to 5 minutes
    while (!gnssFixFlag && millis() - loopStartTime < gnssTimeout * 60UL * 1000UL)
    {
      petDog(); // Reset watchdog timer

      // Check for UBX-NAV-PVT messages
      if (gnss.getPVT())
      {
        // Blink LED
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

        bool dateValidFlag = gnss.getDateValid();
        bool timeValidFlag = gnss.getTimeValid();
        byte fixType = gnss.getFixType();

#if DEBUG_GNSS
        char gnssBuffer[100];
        sprintf(gnssBuffer, "%04u-%02d-%02d %02d:%02d:%02d.%03d,%ld,%ld,%d,%d,%d,%d,%d",
                gnss.getYear(), gnss.getMonth(), gnss.getDay(),
                gnss.getHour(), gnss.getMinute(), gnss.getSecond(), gnss.getMillisecond(),
                gnss.getLatitude(), gnss.getLongitude(), gnss.getSIV(),
                gnss.getPDOP(), fixType, dateValidFlag, timeValidFlag);
        DEBUG_PRINTLN(gnssBuffer);
#endif

        // Check if date and time are valid and sync RTC with GNSS if required
        if (fixType >= 2 && dateValidFlag && timeValidFlag)
        {
          gnssFixFlag = true;                             // Set flag
          rtc.updateTime();                               // Update time variables from RTC
          unsigned long rtcEpoch = rtc.getEpoch();        // Get RTC epoch time
          unsigned long gnssEpoch = gnss.getUnixEpoch();  // Get GNSS epoch time
          rtc.setEpoch(gnssEpoch);                        // Set RTC date and time
          unsigned long rtcDrift = gnssEpoch - rtcEpoch;  // Calculate RTC drift

          // Write data to union
          moMessage.rtcDrift = rtcDrift;
          moMessage.latitude = gnss.getLatitude();
          moMessage.longitude = gnss.getLongitude();
          moMessage.satellites = gnss.getSIV();
          moMessage.pdop = gnss.getPDOP();

          // Change LED colour
          setLedColour(CRGB::Green);

          DEBUG_PRINT("Info: RTC drift: "); DEBUG_PRINTLN(rtcDrift);
          DEBUG_PRINT("Info: RTC time synced to "); printDateTime();
        }
      }
    }
    if (!gnssFixFlag)
    {
      DEBUG_PRINTLN("Warning: Unable to acquire GNSS fix!");
      // Change LED colour
      setLedColour(CRGB::Orange);
    }
  }
  else
  {
    DEBUG_PRINTLN("Warning: u-blox GNSS is offline!");

    setLedColour(CRGB::Red); // Change LED colour
  }

  // Stop the loop timer
  timer.gnss = millis() - loopStartTime;
}
