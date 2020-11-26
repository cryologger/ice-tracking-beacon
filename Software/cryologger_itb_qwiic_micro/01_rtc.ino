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

  rtc.begin(); // Initialize RTC

  // Manually set time and date
  //rtc.setTime(23, 59, 30); // (hours, minutes, seconds)
  //rtc.setDate(21, 11, 20); // (day, month, year)
}

void readRtc() {

  // Start loop timer
  unsigned long loopStartTime = millis();

  // Get UNIX Epoch time
  unixtime = rtc.getEpoch();

  // Write data to union
  message.unixtime = unixtime;

  //SERIAL_PORT.print(F("Datetime: ")); printDateTime();
  //SERIAL_PORT.print(F("UNIX Epoch time: ")); SERIAL_PORT.println(unixtime);

  // Stop loop timer
  unsigned long loopEndTime = millis() - loopStartTime;
  SERIAL_PORT.print(F("readRtc() function execution: ")); SERIAL_PORT.print(loopEndTime); SERIAL_PORT.println(F(" ms"));
}

void setRtcAlarm() {

  // Calculate next alarm
  alarmTime = unixtime + alarmInterval;

  // Check if alarm was set in the past
  if (alarmTime < rtc.getEpoch()) {
    unixtime = rtc.getEpoch(); // Get UNIX Epoch time
    alarmTime = unixtime + alarmInterval; // Recalculate next alarm
    SERIAL_PORT.println(F("Warning: RTC alarm set in the past!"));

    setLedColour(orange);

    // Set alarm to occur on next hour rollover
    rtc.setAlarmTime(0, 0, 0);

    // Enable alarm
    rtc.enableAlarm(rtc.MATCH_MMSS);
  }
  else {

    if (firstTimeFlag) {
      // Set initial alarm to occur on next hour rollover
      rtc.setAlarmTime(0, 0, 0);

      // Enable alarm
      rtc.enableAlarm(rtc.MATCH_MMSS);
      firstTimeFlag = false; // Clear flag
    }
    else {
      // Set alarm time and date
      rtc.setAlarmTime(hour(alarmTime), minute(alarmTime), 0);
      rtc.setAlarmDate(day(alarmTime), month(alarmTime), year(alarmTime) - 2000);

      // Enable alarm
      rtc.enableAlarm(rtc.MATCH_HHMMSS);
    }
  }

  // Attach alarm to interrupt service routine
  rtc.attachInterrupt(alarmIsr);

  // Print the next RTC alarm date and time
  SERIAL_PORT.print("Current time: "); printDateTime();
  SERIAL_PORT.print("Next alarm: "); printAlarm();
}

// Set RTC rolling alarms
void setRtcRollingAlarm() {

  rtc.setAlarmTime((rtc.getHours() + alarmHours) % 24,
                   (rtc.getMinutes() + alarmMinutes) % 60,
                   (rtc.getSeconds() + alarmSeconds) % 60);
  rtc.setAlarmDate(rtc.getDay(), rtc.getMonth(), rtc.getYear());
  rtc.enableAlarm(rtc.MATCH_HHMMSS);
}

// Synchronize RTC with GNSS
void syncRtc() {

  setLedColour(pink);

  // Start loop timer
  unsigned long loopStartTime = millis();

  if (online.gnss) {
    bool dateValid = false;
    bool timeValid = false;
    rtcSyncFlag = false;

    // Attempt to sync RTC with GNSS for up to 5 minutes
    SERIAL_PORT.println(F("Attempting to sync RTC with GNSS..."));

    while ((!dateValid || !timeValid) && millis() - loopStartTime < 5UL * 60UL * 1000UL) {

      dateValid = gps.getDateValid();
      timeValid = gps.getTimeValid();

      // Sync RTC with GNSS if date and time are valid
      if (dateValid && timeValid) {
        rtc.setTime(gps.getHour(), gps.getMinute(), gps.getSecond());
        rtc.setDate(gps.getDay(), gps.getMonth(), gps.getYear() - 2000);
        SERIAL_PORT.print("RTC time synced: "); printDateTime();
        rtcSyncFlag = true;
        setLedColour(green);
      }
      ISBDCallback();
    }
    if (!rtcSyncFlag) {
      SERIAL_PORT.println(F("Warning: RTC sync failed!"));
      setLedColour(red);
    }
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
  SERIAL_PORT.println(dateTimeBuffer);
}

// Print the RTC's alarm
void printAlarm() {
  char alarmBuffer[25];
  sprintf(alarmBuffer, "20%02d-%02d-%02d %02d:%02d:%02d",
          rtc.getAlarmYear(), rtc.getAlarmMonth(), rtc.getAlarmDay(),
          rtc.getAlarmHours(), rtc.getAlarmMinutes(), rtc.getAlarmSeconds());
  SERIAL_PORT.println(alarmBuffer);
}
