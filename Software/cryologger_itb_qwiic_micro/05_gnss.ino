// Configure SparkFun GPS Breakout SAM-M8Q
void configureGnss() {
  blinkLed(1, 1000);
  if (gps.begin()) {
    gps.setI2COutput(COM_TYPE_UBX); // Set I2C port to output UBX only (turn off NMEA noise)
    //gps.saveConfiguration(); // Save current settings to Flash and BBR
    online.gnss = true;
  }
  else {
    SERIAL_PORT.println(F("Warning: SAM-M8Q not detected at default I2C address. Please check wiring."));
    online.gnss = false;
  }
}

// Read SparkFun GPS Breakout SAM-M8Q
void readGnss() {

  setLedColour(cyan);

  if (online.gnss) {

    unsigned long loopStartTime = millis(); // Start loop timer
    gnssFixCounter = 0; // Reset fix counter

    // Look for GNSS signal for up to 5 minutes
    SERIAL_PORT.println(F("Beginning to listen for GNSS traffic..."));

    while ((gnssFixCounter != gnssFixCounterMax) && millis() - loopStartTime < 5UL * 60UL * 1000UL) {

#if DEBUG_GNSS
      char gnssBuffer[100];
      sprintf(gnssBuffer, "%04u-%02d-%02d %02d:%02d:%02d,%ld,%ld,%d,%d,%d",
              gps.getYear(), gps.getMonth(), gps.getDay(),
              gps.getHour(), gps.getMinute(), gps.getSecond(),
              gps.getLatitude(), gps.getLongitude(), gps.getSIV(),
              gps.getFixType(), gps.getPDOP());
      SERIAL_PORT.println(gnssBuffer);
#endif

      // Check for GNSS fix
      if (gps.getFixType() > 0) {
        gnssFixCounter += 1; // Increment counter
      }

      // Check if enough valid GNSS fixes have been collected
      if ((gps.getFixType() > 0) && (gnssFixCounter == gnssFixCounterMax)) {

        SERIAL_PORT.println(F("A GNSS fix was found!"));

        long latitude = gps.getLatitude();
        long longitude = gps.getLongitude();
        byte satellites = gps.getSIV();
        byte fix = gps.getFixType();
        unsigned int pdop = gps.getPDOP();
        bool dateValid = gps.getDateValid();
        bool timeValid = gps.getTimeValid();

        // Write data to union
        message.latitude = latitude;
        message.longitude = longitude;
        message.satellites = satellites;
        message.pdop = pdop;

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
          time_t rtcEpoch = rtc.getEpoch(); // Get current epoch time
          int rtcDrift = rtcEpoch - gnssEpoch; // Calculate difference

          // Write data to union
          message.rtcDrift = rtcDrift;
          SERIAL_PORT.print("RTC drift: "); SERIAL_PORT.println(rtcDrift);

          // Sync RTC date and time
          rtc.setTime(gps.getHour(), gps.getMinute(), gps.getSecond());
          rtc.setDate(gps.getDay(), gps.getMonth(), gps.getYear() - 2000);

          SERIAL_PORT.print("RTC time synced: "); printDateTime();
        }
        setLedColour(green);
      }
      ISBDCallback();
    }

    // Check if a GNSS fix was acquired
    if (gnssFixCounter < gnssFixCounterMax) {
      SERIAL_PORT.println(F("Warning: No GNSS fix was found!"));
      setLedColour(red);
    }

    unsigned long loopEndTime = millis() - loopStartTime; // Stop loop timer
    SERIAL_PORT.print(F("readGnss() function execution: ")); SERIAL_PORT.print(loopEndTime); SERIAL_PORT.println(" ms");
  }
}
