// Configure the real-time clock
void configureRtc() {

  // Alarm modes:
  // 0: Alarm interrupt disabled
  // 1: Alarm match hundredths, seconds, minutes, hour, day, month  (every year)
  // 2: Alarm match hundredths, seconds, minutes, hours, day        (every month)
  // 3: Alarm match hundredths, seconds, minutes, hours, weekday    (every week)
  // 4: Alarm match hundredths, seconds, minute, hours              (every day)
  // 5: Alarm match hundredths, seconds, minutes                    (every hour)
  // 6: Alarm match hundredths, seconds                             (every minute)
  // 7: Alarm match hundredths                                      (every second)

  // Set RTC using the system __DATE__ and __TIME__ macros from compiler
  //rtc.setToCompilerTime();

  // Set RTC date and time
  //rtc.setTime(12, 59, 50, 0, 1, 11, 20); // 2020-11-01 12:59:50.000 (hour, minutes, seconds, hundredths, day, month, year)

  // Set the initial RTC alarm to occur on hour rollover
  rtc.setAlarm(0, 0, 0, 0, 0, 0); // (hour, minutes, seconds, hundredth, day, month)

  // Set the RTC alarm mode
  rtc.setAlarmMode(5); // Alarm match on hundredths, seconds

  // Attach RTC alarm interrupt
  rtc.attachInterrupt();
}

// Read the real-time clock
void readRtc() {

  unsigned long loopStartTime = micros(); // Start loop timer

  // Get RTC's UNIX Epoch time
  unixtime = rtc.getEpoch();

  // Write data to union
  moMessage.unixtime = unixtime;

  //DEBUG_PRINT("Epoch time: "); DEBUG_PRINTLN(unixtime);

  rtcTimer = micros() - loopStartTime;
  //unsigned long loopEndTime = micros() - loopStartTime;
  //DEBUG_PRINT("readRtc() function execution: "); DEBUG_PRINT(loopEndTime); DEBUG_PRINTLN(" us");
}

// Set RTC alarm
void setRtcAlarm()
{
  //(rtc.seconds + alarmSeconds) % 60

  // Calculate next alarm
  alarmTime = unixtime + alarmInterval;

  // Clear the RTC alarm interrupt
  //rtc.clearInterrupt();

  // Clear the RTC alarm interrupt
  am_hal_rtc_int_clear(AM_HAL_RTC_INT_ALM);
  
  // Get the RTC's date and time
  rtc.getTime();

  // Set the RTC's rolling alarm
  //rtc.setAlarm((rtc.hour + alarmHours) % 24,
  //             (rtc.minute + alarmMinutes) % 60,
  //             0,
  //             0, rtc.dayOfMonth, rtc.month);

  rtc.setAlarm(hour(alarmTime), minute(alarmTime), second(alarmTime),
               0, day(alarmTime), month(alarmTime));

  // Set the RTC alarm mode
  rtc.setAlarmMode(5); // Alarm match on hundredths, seconds and minutes

  // Print the next RTC alarm date and time
  DEBUG_PRINT("Next alarm: "); printAlarm();
}

// Print the RTC's current date and time
void printDateTime()
{
  rtc.getTime(); // Get the RTC's date and time
  char dateTimeBuffer[25];
  sprintf(dateTimeBuffer, "20%02d-%02d-%02d %02d:%02d:%02d",
          rtc.year, rtc.month, rtc.dayOfMonth,
          rtc.hour, rtc.minute, rtc.seconds, rtc.hundredths);
  DEBUG_PRINTLN(dateTimeBuffer);
}

// Print the RTC's alarm
void printAlarm()
{
  rtc.getAlarm(); // Get the RTC's date and time
  char alarmBuffer[25];
  sprintf(alarmBuffer, "20%02d-%02d-%02d %02d:%02d:%02d",
          rtc.year, rtc.alarmMonth, rtc.alarmDayOfMonth,
          rtc.alarmHour, rtc.alarmMinute, rtc.alarmSeconds, rtc.alarmHundredths);
  DEBUG_PRINTLN(alarmBuffer);
}
