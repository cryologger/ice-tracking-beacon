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
/*
// Configure real-time clock
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
  moMessage.unixtime = unixtime;

  //DEBUG_PRINT("Datetime: "); printDateTime();
  //DEBUG_PRINT("UNIX Epoch time: "); SERIAL_PORT.println(unixtime);

  // Stop loop timer
  unsigned long loopEndTime = millis() - loopStartTime;
  //DEBUG_PRINT("readRtc() function execution: "); DEBUG_PRINT(loopEndTime); DEBUG_PRINTLN(" ms");
}

void setRtcAlarm() {

  // Calculate next alarm
  alarmTime = unixtime + alarmInterval;

  // Check if alarm was set in the past
  if (alarmTime < rtc.getEpoch()) {
    unixtime = rtc.getEpoch(); // Get UNIX Epoch time
    alarmTime = unixtime + alarmInterval; // Recalculate next alarm
    DEBUG_PRINTLN("Warning: RTC alarm set in the past!");

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
  DEBUG_PRINT("Current time: "); printDateTime();
  DEBUG_PRINT("Next alarm: "); printAlarm();
}

// Set RTC rolling alarms
void setRtcRollingAlarm() {

  rtc.setAlarmTime((rtc.getHours() + alarmHours) % 24,
                   (rtc.getMinutes() + alarmMinutes) % 60,
                   (rtc.getSeconds() + alarmSeconds) % 60);
  rtc.setAlarmDate(rtc.getDay(), rtc.getMonth(), rtc.getYear());
  rtc.enableAlarm(rtc.MATCH_HHMMSS);
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
  DEBUG_PRINTLN(dateTimeBuffer);
}

// Print the RTC's alarm
void printAlarm() {
  char alarmBuffer[25];
  sprintf(alarmBuffer, "20%02d-%02d-%02d %02d:%02d:%02d",
          rtc.getAlarmYear(), rtc.getAlarmMonth(), rtc.getAlarmDay(),
          rtc.getAlarmHours(), rtc.getAlarmMinutes(), rtc.getAlarmSeconds());
  DEBUG_PRINTLN(alarmBuffer);
}
*/


/*
  // Test the signal quality
  int signalQuality = -1;
  err = modem.getSignalQuality(signalQuality);
  if (err != ISBD_SUCCESS)
  {
    DEBUG_PRINT("Warning: Signal quality failed with error ");
    DEBUG_PRINTLN(err);
    return;
  }
  DEBUG_PRINT("On a scale of 0 to 5, signal quality is currently: ");
  DEBUG_PRINTLN(signalQuality);
*/

    
