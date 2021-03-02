void configureGnss()
{
  if (gnss.begin())
  {
    gnss.setI2COutput(COM_TYPE_UBX);                  // Set I2C port to output UBX only
    gnss.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);  // Save communications port settings to flash and BBR
    gnss.setNavigationFrequency(2);                   // Produce two solutions per second
    //gnss.enableDebugging();                         // Uncomment this line to enable debug messages on Serial
    gnss.setAutoPVTcallback(&getGnssFix);             // Enable automatic NAV PVT messages with callback to printPVTdata
    online.gnss = true;
  }
  else
  {
    DEBUG_PRINTLN("Warning: u-blox GNSS not detected at default I2C address. Please check wiring.");
    online.gnss = false;
  }
}
//createFileBuffer: Warning. FileBufferSize is zero. Data logging is not possible.

// Read the GNSS receiver
void readGnss()
{
  // Start loop timer
  unsigned long loopStartTime = millis();

  rtcSyncFlag = false; // Clear flag
  gnssFixFlag = false; // Clear flag
  gnssFixCounter = 0; // Reset counter

  // Check if GNSS initialized successfully
  if (online.gnss)
  {
    DEBUG_PRINTLN("Acquiring GNSS fix...");
    
    setLedColour(CRGB::Cyan); // Change LED colour

    // Attempt to acquire a valid GNSS position fix
    while (!gnssFixFlag && millis() - loopStartTime < gnssTimeout * 1000UL)
    {
      gnss.checkUblox(); // Check for the arrival of new data and process it
      gnss.checkCallbacks(); // Check if any callbacks are waiting to be processed
    }
    if (!gnssFixFlag)
    {
      DEBUG_PRINTLN("Warning: Unable to acquire GNSS fix!");
    }
  }
  else
  {
    DEBUG_PRINTLN("Warning: u-blox GNSS offline!");
  }

  // Turn off LED
  digitalWrite(LED_BUILTIN, LOW);

  // Stop the loop timer
  timer.gnss = millis() - loopStartTime;
}

// Callback function to process UBX-NAV-PVT data
void getGnssFix(UBX_NAV_PVT_data_t ubx)
{
  // Reset the Watchdog Timer
  petDog();

  // Read battery voltage
  readBattery();

  // Blink LED
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  bool dateValidFlag = ubx.valid.bits.validDate;
  bool timeValidFlag = ubx.valid.bits.validTime;
  byte fixType = ubx.fixType;

#if DEBUG_GNSS
  char gnssBuffer[100];
  sprintf(gnssBuffer, "%04u-%02d-%02d %02d:%02d:%02d.%03d,%ld,%ld,%d,%d,%d,%d,%d",
          ubx.year, ubx.month, ubx.day,
          ubx.hour, ubx.min, ubx.sec, ubx.iTOW % 1000,
          ubx.lat, ubx.lon, ubx.numSV,
          ubx.pDOP, ubx.fixType,
          ubx.valid.bits.validDate, ubx.valid.bits.validTime);
  DEBUG_PRINTLN(gnssBuffer);
#endif

  // Check if GNSS fix is valid
  if (fixType == 3)
  {
    gnssFixCounter += 2; // Increment counter
  }
  else if (fixType == 2)
  {
    gnssFixCounter += 1; // Increment counter
  }

  // Check if GNSS fix threshold has been reached
  if (gnssFixCounter >= gnssFixCounterMax)
  {
    DEBUG_PRINTLN("A GNSS fix was found!");
    gnssFixFlag = true; // Set fix flag

    // Write data to union
    moMessage.latitude = ubx.lat;
    moMessage.longitude = ubx.lon;
    moMessage.satellites = ubx.numSV;
    moMessage.pdop = ubx.pDOP;

    // Attempt to sync RTC with GNSS
    if (fixType >= 2 && timeValidFlag && dateValidFlag)
    {
      // Convert GNSS date and time to Unix Epoch time
      tmElements_t tm;
      tm.Year = ubx.year - 1970;
      tm.Month = ubx.month;
      tm.Day = ubx.day;
      tm.Hour = ubx.hour;
      tm.Minute = ubx.min;
      tm.Second = ubx.sec;
      time_t gnssEpoch = makeTime(tm);
      rtc.updateTime(); // Update time variables from RTC

      // Calculate drift
      int rtcDrift = rtc.getEpoch() - gnssEpoch;

      DEBUG_PRINT("RTC drift: "); DEBUG_PRINTLN(rtcDrift);

      // Write data to union
      moMessage.rtcDrift = rtcDrift;

      // Set RTC date and time
      rtc.setTime(ubx.sec, ubx.min, ubx.hour, 0,
                  ubx.day, ubx.month, ubx.year);

                  
      rtcSyncFlag = true; // Set flag
      DEBUG_PRINT("RTC time synced to: "); printDateTime();
    }
    else
    {
      DEBUG_PRINTLN("Warning: RTC sync not performed due to invalid GNSS fix!");
    }
  }
}
