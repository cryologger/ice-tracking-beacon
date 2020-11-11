// Configure SparkFun GPS Breakout SAM-M8Q
void configureGnss() {

  if (gps.begin()) {
    gps.setI2COutput(COM_TYPE_UBX); // Set I2C port to output UBX only (turn off NMEA noise)
    gps.saveConfiguration();        // Save current settings to Flash and BBR
    online.gnss = true;
  }
  else {
    Serial.println(F("Warning: u-blox SAM-M8Q not detected at default I2C address. Please check wiring."));
    online.gnss = false;
    //while (1);
  }
}

// Read the GPS Breakout SAM-M8Q
void readGnss() {

  unsigned long loopStartTime = millis(); // Loop timer

  // Begin listening to the GNSS
  Serial.println(F("Beginning to listen for GNSS traffic..."));

  // Look for GNSS signal for up to 5 minutes
  while ((valFix != maxValFix) && millis() - loopStartTime < 10UL * 60UL * 1000UL) {

#if DEBUG
    Serial.printf("%04u-%02d-%02d %02d:%02d:%02d,%ld,%ld,%d,%d,%d\n",
                  gps.getYear(), gps.getMonth(), gps.getDay(),
                  gps.getHour(), gps.getMinute(), gps.getSecond(),
                  gps.getLatitude(), gps.getLongitude(), gps.getSIV(),
                  gps.getFixType(), gps.getPDOP());
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

      // Write data to SD buffer
      sprintf(tempData, "%ld,%ld,%d,%d,%d,\n", latitude, longitude, satellites, fix, pdop);
      strcat(outputData, tempData);

      // Write data to union
      message.latitude = latitude;
      message.longitude = longitude;
      message.satellites = satellites;
      message.fix = fix;
      message.pdop = pdop;
      break;

      // Sync RTC with GNSS if date and time are valid
      if (gps.getDateValid() && gps.getTimeValid()) {
        rtc.setTime(gps.getHour(), gps.getMinute(), gps.getSecond(), gps.getMillisecond() / 10,
                    gps.getDay(), gps.getMonth(), gps.getYear() - 2000);
        Serial.print("RTC time synced: "); printDateTime();
      }

    }
    blinkLed(1, 500);
    petDog(); // Reset Watchdog Timer
  }

  // Check if a GNSS fix was acquired
  if (valFix < maxValFix) {
    Serial.println(F("Warning: No GNSS fix was found"));

    // Write empty data to SD buffer
    sprintf(tempData, "%ld,%ld,%d,%d,%d,\n", 0, 0, 0, 0, 0);
    strcat(outputData, tempData);
  }

  // Reset valFix counter
  valFix = 0;

  unsigned long loopEndTime = millis() - loopStartTime;
  Serial.printf("readGnss() function execution: %d ms\n", loopEndTime);

}
