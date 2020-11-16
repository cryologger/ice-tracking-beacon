void configureRtc() {

  // Initialize RTC
  rtc.begin();

  // Manually set date and time
  //rtc.setTime(12, 55, 0); // hours, minutes, seconds
  //rtc.setDate(11, 11, 20); // day, month, year

  // Set initial alarm to occur on seconds rollover
  rtc.setAlarmTime(0, 0, 0);
  rtc.setAlarmDate(0, 0, 0);
  rtc.enableAlarm(rtc.MATCH_SS);

  // Attach alarm to interrupt service routine
  rtc.attachInterrupt(alarmIsr);

  // Print initial datetime and alarm
  printDateTime();
  printAlarm();
}

void readRtc() {

  unsigned long loopStartTime = millis(); // Loop timer

  // Get UNIX Epoch time
  unixtime = rtc.getEpoch();

  // Write data to union
  message.unixtime = unixtime;

  Serial.print(F("Datetime: ")); printDateTime();
  Serial.print(F("UNIX Epoch time: ")); Serial.println(unixtime);

  unsigned long loopEndTime = millis() - loopStartTime;
  Serial.print(F("readRtc() function execution: ")); Serial.print(loopEndTime); Serial.println(F(" ms"));
}

void setRtcAlarm() {

  // Set the RTC's rolling alarm
  rtc.setAlarmTime((rtc.getHours() + alarmHours) % 24,
                   (rtc.getMinutes() + alarmMinutes) % 60,
                   0);
  rtc.setAlarmDate(rtc.getDay(), rtc.getMonth(), rtc.getYear());
  rtc.enableAlarm(rtc.MATCH_HHMMSS);

  // Print the next RTC alarm date and time
  Serial.print("Next alarm: "); printAlarm();
}

void syncRtc() {

  unsigned long loopStartTime = millis(); // Loop timer
  bool dateValid = false;
  bool timeValid = false;
  rtcSyncFlag = false;

  // Attempt to sync RTC with GNSS for up to 5 minutes
  Serial.println(F("Attempting to sync RTC with GNSS..."));

  while ((!dateValid || !timeValid) && millis() - loopStartTime < 1UL * 10UL * 1000UL) {

    dateValid = gps.getDateValid();
    timeValid = gps.getTimeValid();

    // Sync RTC with GNSS if date and time are valid
    if (dateValid && timeValid) {
      rtc.setTime(gps.getHour(), gps.getMinute(), gps.getSecond());
      rtc.setDate(gps.getDay(), gps.getMonth(), gps.getYear() - 2000);
      Serial.print("RTC time synced: "); printDateTime();
      blinkLed(10, 50); // Blink LED to indicate RTC sync
    }
    ISBDCallback();
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
