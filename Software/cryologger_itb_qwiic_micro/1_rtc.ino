void configureRtc() {
  rtc.begin();

  //rtc.setTime(12, 55, 0); // hours, minutes, seconds
  //rtc.setDate(11, 11, 20); // day, month, year

  rtc.setAlarmTime(0, 0, 0);
  rtc.enableAlarm(rtc.MATCH_SS);

  rtc.attachInterrupt(alarmIsr);

  printDateTime();
  printAlarm();
}

void readRtc() {

  unixtime = rtc.getEpoch();

  // Write data to union
  message.unixtime = unixtime;

  Serial.print(F("Datetime: ")); printDateTime();
  Serial.print(F("UNIX Epoch time: ")); Serial.println(unixtime);
}

void syncRtc() {

  unsigned long loopStartTime = millis(); // Loop timer
  bool dateValid = false;
  bool timeValid = false;
  rtcSyncFlag = false;

  // Attempt to sync RTC with GNSS for up to 5 minutes
  Serial.println(F("Attempting to sync RTC with GNSS..."));

  while ((!dateValid  || !timeValid) && millis() - loopStartTime < 1UL * 60UL * 1000UL) {

    dateValid = gps.getDateValid();
    timeValid = gps.getTimeValid();

    // Sync RTC if GNSS date and time are valid
    if (dateValid && timeValid) {
      rtc.setTime(gps.getHour(), gps.getMinute(), gps.getSecond());
      rtc.setDate(gps.getDay(), gps.getMonth(), gps.getYear() - 2000);
      rtcSyncFlag = true; // Set flag
      blinkLed(10, 50);
      Serial.print(F("RTC time synced: ")); printDateTime();
    }

    blinkLed(1, 500);
    //ISBDCallback();
  }
  if (!rtcSyncFlag) {
    Serial.println(F("Warning: RTC sync failed"));
  }
}

// RTC alarm interrupt service routine
void alarmIsr() {
  alarmFlag = true; // Set alarm flag
}

// Print the RTC's current date and time
void printDateTime() {
  char dateTimeBuffer[25];
  sprintf(dateTimeBuffer, "20%02d-%02d-%02d %02d:%02d:%02d",
          rtc.getYear(), rtc.getMonth(), rtc.getDay(),
          rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
  Serial.println(dateTimeBuffer);
}

// Print the RTC's alarm
void printAlarm() {
  char alarmBuffer[25];
  sprintf(alarmBuffer, "20%02d-%02d-%02d %02d:%02d:%02d",
          rtc.getAlarmYear(), rtc.getAlarmMonth(), rtc.getAlarmDay(),
          rtc.getAlarmHours(), rtc.getAlarmMinutes(), rtc.getAlarmSeconds());
  Serial.println(alarmBuffer);
}



/*
  // Convert date and time to Unix Epoch time
  uint32_t getEpoch() {

  // Update the local array with the RTC registers
  rtc.updateTime();

  // Convert to Unix epoch (tm to time_t)
  tmElements_t tm;
  tm.Second = rtc.getSeconds();
  tm.Minute = rtc.getMinutes();
  tm.Hour   = rtc.getHours();
  tm.Day    = rtc.getDate();
  tm.Month  = rtc.getMonth();
  tm.Year   = rtc.getYear() + 30; // Offset from 2000 - 1970
  time_t t = makeTime(tm);

  return t;
  }
