/*
   RTC Alarm Modes:
   MATCH_OFF            // Never
   MATCH_SS             // Every Minute
   MATCH_MMSS           // Every Hour
   MATCH_HHMMSS         // Every Day
   MATCH_DHHMMSS        // Every Month
   MATCH_MMDDHHMMSS     // Every Year
   MATCH_YYMMDDHHMMSS   // Once, on a specific date and a specific time
*/

void configureRtc() {
  // Initialize RTC
  rtc.begin();

  // Manually set time and date
  rtc.setTime(23, 59, 30); // (hours, minutes, seconds)
  rtc.setDate(21, 11, 20); // (day, month, year)
}

void readRtc() {

  // Start loop timer
  unsigned long loopStartTime = millis();

  // Get UNIX Epoch time
  unixtime = rtc.getEpoch();

  // Write data to union
  message.unixtime = unixtime;

  //Serial.print(F("Datetime: ")); printDateTime();
  //Serial.print(F("UNIX Epoch time: ")); Serial.println(unixtime);

  unsigned long loopEndTime = millis() - loopStartTime;
  Serial.print(F("readRtc() function execution: ")); Serial.print(loopEndTime); Serial.println(F(" ms"));
}

void setRtcAlarm() {

  // Calculate next alarm
  alarmTime = unixtime + alarmInterval;

  // Check if alarm was set in the past
  if (alarmTime < rtc.getEpoch()) {
    unixtime = rtc.getEpoch(); // Get UNIX Epoch time
    alarmTime = unixtime + alarmInterval; // Recalculate next alarm
    Serial.println(F("Warning: RTC alarm set in the past"));
  }

  // Set alarm time and date
  rtc.setAlarmTime(hour(alarmTime), minute(alarmTime), 0);
  rtc.setAlarmDate(day(alarmTime), month(alarmTime), year(alarmTime) - 2000);

  // Enable alarm
  rtc.enableAlarm(rtc.MATCH_HHMMSS);

  // Attach alarm to interrupt service routine
  rtc.attachInterrupt(alarmIsr);

  // Print the next RTC alarm date and time
  Serial.print("Current time: "); printDateTime();
  Serial.print("Next alarm: "); printAlarm();
}

// Set RTC rolling alarms
void setRtcRollingAlarm() {

  rtc.setAlarmTime((rtc.getHours() + alarmHours) % 24,
                   (rtc.getMinutes() + alarmMinutes) % 60,
                   (rtc.getSeconds() + alarmSeconds) % 60);
  rtc.setAlarmDate(rtc.getDay(), rtc.getMonth(), rtc.getYear());
  rtc.enableAlarm(rtc.MATCH_HHMMSS);
}
void syncRtc() {

  setPixelColour(pink);
  if (online.gnss) {
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
        rtcSyncFlag = true;
        setPixelColour(green);
      }
      ISBDCallback();
    }
    if (!rtcSyncFlag) {
      Serial.println(F("Warning: RTC sync failed"));
      setPixelColour(red);
    }
  }
  // Set initial alarm to occur on hour rollover
  rtc.setAlarmTime(0, 0, 0);
  rtc.setAlarmDate(0, 0, 0);
  //rtc.enableAlarm(rtc.MATCH_MMSS); // Hours rollover
  rtc.enableAlarm(rtc.MATCH_SS); // Minutes rollovr

  // Attach alarm to interrupt service routine
  rtc.attachInterrupt(alarmIsr);

  // Print initial datetime and alarm
  Serial.print("Datetime: "); printTab(1); printDateTime();
  Serial.print("Alarm: "); printTab(2); printAlarm();
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
