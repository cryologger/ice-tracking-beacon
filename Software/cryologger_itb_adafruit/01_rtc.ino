// Configure the real-time clock
void configureRtc()
{
  // Alarm modes:
  // MATCH_OFF          Never
  // MATCH_SS           Every Minute
  // MATCH_MMSS         Every Hour
  // MATCH_HHMMSS       Every Day
  // MATCH_DHHMMSS      Every Month
  // MATCH_MMDDHHMMSS   Every Year
  // MATCH_YYMMDDHHMMSS Once, on a specific date and a specific time

  rtc.begin();

  //rtc.setTime(hours, minutes, seconds);
  //rtc.setDate(day, month, year);

  rtc.setAlarmTime(17, 00, 10);
  rtc.enableAlarm(rtc.MATCH_MMSS);

  rtc.attachInterrupt(alarmIsr);

}

void readRtc()
{

}

void setInitialAlarm()
{

}

void setRtcAlarm()
{

}

// RTC alarm interrupt service routine (ISR)
void alarmIsr()
{
  alarmFlag = true;
}


// Print the RTC's current date and time
void printDateTime()
{
  char dateTimeBuffer[25];
  sprintf(dateTimeBuffer, "%04d-%02d-%02d %02d:%02d:%02d",
          rtc.getYear(), rtc.getMonth(), rtc.getDay(),
          rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
  DEBUG_PRINTLN(dateTimeBuffer);
}

// Print the RTC's alarm
void printAlarm()
{
  char alarmBuffer[25];
  sprintf(alarmBuffer, "%04d-%02d-%02d %02d:%02d:00",
          rtc.getAlarmYear(), rtc.getAlarmMonth(), rtc.getAlarmDay(),
          rtc.getAlarmHours(), rtc.getAlarmMinutes(), rtc.getAlarmSeconds());
  DEBUG_PRINTLN(alarmBuffer);
}
