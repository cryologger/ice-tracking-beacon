// Read GPS
void readGps()
{
  unsigned long loopStartTime = millis();
  bool fixFound = false;
  bool charsSeen = false;

  // Enable power to GPS
  enableGpsPower();

  Serial.println("Beginning to listen for GPS traffic...");
  GPS_PORT.begin(9600);
  myDelay(1000);

  // Configure GPS
  GPS_PORT.println("$PMTK220,1000*1F");                                  // Set NMEA port update rate to 1Hz
  delay(100);
  GPS_PORT.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"); // Set NMEA sentence output frequencies to GGA and RMC
  delay(100);
  //GpsSerial.println("$PGCMD,33,1*6C");                                    // Enable antenna updates
  GPS_PORT.println("$PGCMD,33,0*6D");                                    // Disable antenna updates

  // Look for GPS signal for up to 2 minutes
  while (!fixFound && millis() - loopStartTime < gpsTimeout * 60UL * 1000UL)
  {
    if (GPS_PORT.available())
    {
      charsSeen = true;
      char c = GPS_PORT.read();
      Serial.write(c); // Echo NMEA sentences to serial
      if (gps.encode(c))
      {
        if ((gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) &&
            (gps.location.isUpdated() && gps.date.isUpdated() && gps.time.isUpdated()))
        {
          fixFound = true;
          moMessage.latitude = gps.location.lat() * 1000000;
          moMessage.longitude = gps.location.lng() * 1000000;
          moMessage.satellites = gps.satellites.value();
          moMessage.hdop = gps.hdop.value();

          // Sync real-time clock (RTC) with GPS time
          tm.Hour = gps.time.hour();
          tm.Minute = gps.time.minute();
          tm.Second = gps.time.second();
          tm.Day = gps.date.day();
          tm.Month = gps.date.month();
          tm.Year = gps.date.year() - 1970;   // tmElements_t.Year is the offset from 1970
          unsigned long gpsTime = makeTime(tm);      // Change the tm structure into time_t (seconds since epoch)

          // Compare current date and time of GPS and real-time clock
          Serial.println();
          Serial.print(F("RTC time: "));
          Serial.print(rtc.getEpoch());
          Serial.print(F("GPS time: "));
          Serial.println(gpsTime);
          unsigned long drift = rtc.getEpoch() - gpsTime;
          Serial.print(F("Drift: ")); Serial.print(drift); Serial.print(F(" seconds"));

          // Set real-time clock if drift exceeds 30 seconds
          if (drift > 15 || drift < -15)
          {
            rtc.setEpoch(gpsTime);
            Serial.print(F("Real-time clock synced with GPS date and time"));
          }
        }
      }
    }

    ISBDCallback();

    if ((millis() - loopStartTime) > 5000 && gps.charsProcessed() < 10)
    {
      Serial.println(F("No GPS data received: check wiring"));
      break;
    }
  }
  Serial.println(charsSeen ? fixFound ? F("A GPS fix was found!") : F("No GPS fix was found.") : F("Wiring error: No GPS data seen."));

  // Disable power to GPS
  disableGpsPower();
  
  // Stop the loop timer
  timer.readGps = millis() - loopStartTime;
}
