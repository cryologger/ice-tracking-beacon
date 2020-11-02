void configureRtc() {

  // Set RTC using the system __DATE__ and __TIME__ macros from compiler
  //rtc.setToCompilerTime();

  // Set RTC date and time
  //rtc.setTime(12, 59, 50, 0, 1, 11, 20); // 2020-11-01 12:59:50.000 (hour, minutes, seconds, hundredths, day, month, year)

  // Set the RTC alarm
  rtc.setAlarm(0, 0, 0, 0, 0, 0); // (hour, minutes, seconds, hundredth, day, month)
  /*
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
  Serial.print(F("Alarm: "));
  printAlarm();
}

void syncRtc()
{
  unsigned long loopStartTime = millis(); // Loop timer
  bool dateValid = false;
  bool timeValid = false;
  bool rtcSyncFlag = false;

  // Attempt to sync RTC with GNSS for up to 5 minutes
  Serial.println(F("Attempting to sync RTC with GNSS..."));

  while ((dateValid == false || timeValid == false) && millis() - loopStartTime < 5UL * 60UL * 1000UL)
  {
    dateValid = gps.getDateValid();
    timeValid = gps.getTimeValid();

    // Sync RTC if GNSS date and time are valid
    if (dateValid && timeValid) {
      rtc.setTime(gps.getHour(),
                  gps.getMinute(),
                  gps.getSecond(),
                  gps.getMillisecond() / 10,
                  gps.getDay(),
                  gps.getMonth(),
                  gps.getYear() - 2000);

      rtcSyncFlag = true; // Set flag
      Serial.println(F("RTC time synced:"));
      printDateTime();
    }
    
    ISBDCallback();
  }
  if (rtcSyncFlag == false) {
    Serial.println(F("Warning: RTC sync failed"));
  }
}

// Print the RTC's current date and time
void printDateTime()
{
  rtc.getTime();
  char dateTimeBuffer[25];
  sprintf(dateTimeBuffer, "20%02d-%02d-%02d %02d:%02d:%02d.%03d",
          rtc.year, rtc.month, rtc.dayOfMonth,
          rtc.hour, rtc.minute, rtc.seconds, rtc.hundredths);
  Serial.println(dateTimeBuffer);
}

// Print the RTC's alarm
void printAlarm()
{
  rtc.getAlarm();
  char alarmBuffer[25];
  sprintf(alarmBuffer, "2020-%02d-%02d %02d:%02d:%02d.%03d",
          rtc.alarmMonth, rtc.alarmDayOfMonth,
          rtc.alarmHour, rtc.alarmMinute,
          rtc.alarmSeconds, rtc.alarmHundredths);
  Serial.println(alarmBuffer);
}
