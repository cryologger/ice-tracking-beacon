void configureRtc() {

  // Set RTC using the system __DATE__ and __TIME__ macros from compiler
  //rtc.setToCompilerTime();

  // Set RTC date and time
  //rtc.setTime(12, 59, 50, 0, 1, 11, 20); // 2020-11-01 12:59:50.000 (hour, minutes, seconds, hundredths, day, month, year)

  // Set the RTC alarm
  rtc.setAlarm(0, 5, 0, 0, 0, 0); // (hour, minutes, seconds, hundredth, day, month)
  /*
    Alarm modes:
    0: Alarm interrupt disabled
    1: Alarm match every year   (hundredths, seconds, minutes, hour, day, month)
    2: Alarm match every month  (hundredths, seconds, minutes, hours, day)
    3: Alarm match every week   (hundredths, seconds, minutes, hours, weekday)
    4: Alarm match every day    (hundredths, seconds, minute, hours)
    5: Alarm match every hour   (hundredths, seconds, minutes)
    6: Alarm match every minute (hundredths, seconds)
    7: Alarm match every second (hundredths)
  */
  rtc.setAlarmMode(6); // Set the RTC alarm mode
  rtc.attachInterrupt(); // Attach RTC alarm interrupt
  DEBUG_PRINT("Alarm: "); printAlarm();
}

void readRtc() {

  unsigned long loopStartTime = micros(); // Loop timer

  // Get UNIX Epoch time
  unsigned long unixtime = rtc.getEpoch();

  // Write data to union
  moMessage.unixtime = unixtime;

  DEBUG_PRINT("readRtc(): "); printDateTime();
  //DEBUG_PRINT("Epoch time: "); DEBUG_PRINTLN(unixtime);

  unsigned long loopEndTime = micros() - loopStartTime;
  //DEBUG_PRINT("readRtc() function execution: "); DEBUG_PRINT(loopEndTime); DEBUG_PRINTLN(" us");
}

// Sync RTC date and time with GNSS
void setRtcAlarm() {

  rtc.getTime();

  // Set the RTC's rolling alarm
  rtc.setAlarm((rtc.hour + alarmHours) % 24,
               (rtc.minute + alarmMinutes) % 60,
               0,
               0, rtc.dayOfMonth, rtc.month);
  rtc.setAlarmMode(5);
  //(rtc.seconds + alarmSeconds) % 60
  // Print the next RTC alarm date and time
  DEBUG_PRINT("Next alarm: "); printAlarm();
}

// Print the RTC's current date and time
void printDateTime() {
  rtc.getTime();
  char dateTimeBuffer[25];
  sprintf(dateTimeBuffer, "20%02d-%02d-%02d %02d:%02d:%02d",
          rtc.year, rtc.month, rtc.dayOfMonth,
          rtc.hour, rtc.minute, rtc.seconds, rtc.hundredths);
  DEBUG_PRINTLN(dateTimeBuffer);
}

// Print the RTC's alarm
void printAlarm() {
  rtc.getAlarm();
  char alarmBuffer[25];
  sprintf(alarmBuffer, "20%02d-%02d-%02d %02d:%02d:%02d",
          rtc.year, rtc.alarmMonth, rtc.alarmDayOfMonth,
          rtc.alarmHour, rtc.alarmMinute, rtc.alarmSeconds, rtc.alarmHundredths);
  DEBUG_PRINTLN(alarmBuffer);
}
