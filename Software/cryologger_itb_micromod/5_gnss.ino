
// Configure SparkFun GPS Breakout SAM-M8Q
void configureGnss() {
    
  if (gps.begin()) {
    gps.setI2COutput(COM_TYPE_UBX); // Set I2C port to output UBX only (turn off NMEA noise)
    gps.saveConfiguration();        // Save current settings to Flash and BBR
  }
  else {
    Serial.println(F("Warning: u-blox SAM-M8Q not detected at default I2C address. Please check wiring."));
    //while (1);
  }
}

// Read the SparkFun u-blox SAM-M8Q
void readGnss() {

  unsigned long loopStartTime = millis(); // Loop timer

  // Begin listening to the GNSS
  Serial.println(F("Beginning to listen for GNSS traffic..."));

  // Look for GNSS signal for up to 5 minutes
  while ((valFix != maxValFix) && millis() - loopStartTime < 1UL * 60UL * 1000UL) {
#if DEBUG
    char datetime[20]; // GNSS datetime buffer
    snprintf(datetime, sizeof(datetime), "%04u-%02d-%02d %02d:%02d:%02d",
             gps.getYear(), gps.getMonth(), gps.getDay(),
             gps.getHour(), gps.getMinute(), gps.getSecond());

    float latitude = gps.getLatitude() / 10000000.0;
    float longitude = gps.getLongitude() / 10000000.0;
    float pdop = gps.getPDOP() / 100.0;
    uint8_t fix = gps.getFixType();
    uint8_t satellites = gps.getSIV();

    Serial.print(datetime);
    Serial.print(F(",")); Serial.print(latitude, 6);
    Serial.print(F(",")); Serial.print(longitude, 6);
    Serial.print(F(",")); Serial.print(satellites);
    Serial.print(F(",")); Serial.print(fix);
    Serial.print(F(",")); Serial.println(pdop, 2);
#endif

    // Did we get a GPS fix?
    if (gps.getFixType() > 0) {
      valFix += 1; // Increment valFix
    }

    // Have enough valid GNSS fixes been collected?
    if (valFix == maxValFix) {
      Serial.println(F("A GNSS fix was found!"));

      // Sync RTC with GNSS date and time
      rtc.setTime(gps.getHour(),
                  gps.getMinute(),
                  gps.getSecond(),
                  gps.getMillisecond() / 10,
                  gps.getDay(),
                  gps.getMonth(),
                  gps.getYear() - 2000);

      Serial.print("RTC time synced: ");
      printDateTime();

      // Write data to union
      message.latitude = gps.getLatitude();
      message.longitude = gps.getLongitude();
      message.satellites = gps.getSIV();
      message.pdop = gps.getPDOP();
      message.fix = gps.getFixType();
      break;
    }
    ISBDCallback();
  }

  // Did we get a GNSS fix?
  if (valFix < maxValFix) {
    Serial.println(F("Warning: No GNSS fix was found"));
  }

  // Reset valFix counter
  valFix = 0;

  unsigned long loopEndTime = millis() - loopStartTime;
  Serial.print(F("readGnss() function execution: ")); Serial.print(loopEndTime); Serial.println(F(" ms"));

}
