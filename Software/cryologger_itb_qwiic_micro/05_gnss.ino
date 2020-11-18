// Configure SparkFun GPS Breakout SAM-M8Q
void configureGnss() {
  blinkLed(1, 1000);
  if (gps.begin()) {
    gps.setI2COutput(COM_TYPE_UBX); // Set I2C port to output UBX only (turn off NMEA noise)
    //gps.saveConfiguration();        // Save current settings to Flash and BBR
    online.gnss = true;
  }
  else {
    Serial.println(F("Warning: SAM-M8Q not detected at default I2C address. Please check wiring."));
    online.gnss = false;
  }
}

// Read SparkFun GPS Breakout SAM-M8Q
void readGnss() {

  setPixelColour(cyan);

  if (online.gnss) {

    unsigned long loopStartTime = millis(); // Loop timer
    valFix = 0; // Reset fix counter

    // Begin listening to the GNSS
    Serial.println(F("Beginning to listen for GNSS traffic..."));

    blinkLed(2, 1000); // Non-blocking delay to allow GNSS receiver to boot
    // Look for GNSS signal for up to 5 minutes
    while ((valFix != maxValFix) && millis() - loopStartTime < 5UL * 60UL * 1000UL) {

#if DEBUG
      char gnssBuffer[100];
      sprintf(gnssBuffer, "%04u-%02d-%02d %02d:%02d:%02d,%ld,%ld,%d,%d,%d",
              gps.getYear(), gps.getMonth(), gps.getDay(),
              gps.getHour(), gps.getMinute(), gps.getSecond(),
              gps.getLatitude(), gps.getLongitude(), gps.getSIV(),
              gps.getFixType(), gps.getPDOP());
      Serial.println(gnssBuffer);
#endif

      // Check for GNSS fix
      if (gps.getFixType() > 0) {
        valFix += 1; // Increment counter
      }

      // Check if enough valid GNSS fixes have been collected
      if ((gps.getFixType() > 0) && (valFix == maxValFix)) {

        Serial.println(F("A GNSS fix was found!"));

        // Record GNSS coordinates
        long latitude = gps.getLatitude();
        long longitude = gps.getLongitude();
        byte satellites = gps.getSIV();
        byte fix = gps.getFixType();
        unsigned int pdop = gps.getPDOP();

        // Write data to union
        message.latitude = latitude;
        message.longitude = longitude;
        message.satellites = satellites;
        message.pdop = pdop;
        break;

        // Sync RTC with GNSS if date and time are valid
        if (gps.getDateValid() && gps.getTimeValid()) {
          rtc.setTime(gps.getHour(), gps.getMinute(), gps.getSecond());
          rtc.setDate(gps.getDay(), gps.getMonth(), gps.getYear() - 2000);
          Serial.print("RTC time synced: "); printDateTime();
        }
        setPixelColour(green);
      }
      ISBDCallback();
    }

    // Check if a GNSS fix was acquired
    if (valFix < maxValFix) {
      Serial.println(F("Warning: No GNSS fix was found"));
      setPixelColour(red);
    }

    unsigned long loopEndTime = millis() - loopStartTime;
    Serial.print(F("readGnss() function execution: ")); Serial.print(loopEndTime); Serial.println(" ms");
  }
}
