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

  // Initialize RTC
  rtc.begin();

  // Set time manually
  //rtc.setTime(hours, minutes, seconds);
  //rtc.setDate(day, month, year);
  //rtc.setEpoch();

  // Set initial RTC alarm time
  rtc.setAlarmTime(0, 0, 0); // hours, minutes, seconds

  // Enable alarm for hour rollover match
  rtc.enableAlarm(rtc.MATCH_MMSS);

  // Attach alarm interrupt service routine (ISR)
  rtc.attachInterrupt(alarmIsr);

  alarmFlag = false; // Clear flag

  DEBUG_PRINT("Info: RTC initialized "); printDateTime();
  DEBUG_PRINT("Info: Initial alarm "); printAlarm();
}

// Read RTC
void readRtc()
{
  uint32_t loopStartTime = millis();

  // Get Unix Epoch time
  unixtime = rtc.getEpoch();

  // Write data to union
  moMessage.unixtime = unixtime;

  // Stop loop timer
  timer.rtc = millis() - loopStartTime;
}

// Set RTC alarm
void setRtcAlarm()
{
  // Set RTC alarm according to number of consecutive failed transmissions
  if (failedTransmitCounter < 5)
  {
    // Calculate next alarm
    alarmTime = unixtime + alarmInterval;
  }
  else if (failedTransmitCounter > 5 && failedTransmitCounter < 10)
  {
    // Add a 12-hour delay to next transmission attempt
    alarmTime = unixtime + alarmInterval + 43200;
  }
  else if (failedTransmitCounter > 10)
  {
    // Add a 24-hour delay to next transmission attempt
    alarmTime = unixtime + alarmInterval + 86400;
  }
  else if (failedTransmitCounter > 20)
  {
    // Add a 48-hour delay to next transmission attempt
    alarmTime = unixtime + alarmInterval + 172800;
  }

  // Check if alarm was set in the past
  if ((rtc.getEpoch() >= alarmTime) || firstTimeFlag)
  {
    DEBUG_PRINTLN(F("Warning: RTC alarm set in the past or program running for the first time."));

    // Set alarm for hour rollover match
    rtc.setAlarmTime(0, 0, 0); // hours, minutes, seconds

    // Enable alarm for hour rollover match
    rtc.enableAlarm(rtc.MATCH_MMSS);
  }
  else
  {
    DEBUG_PRINTLN(F("Info: Setting RTC alarm based on specified interval."));


    // Set alarm time
    rtc.setAlarmTime(hour(alarmTime), minute(alarmTime), 0); // hours, minutes, seconds

    // Set alarm date
    rtc.setAlarmDate(day(alarmTime), month(alarmTime), year(alarmTime) - 2000);

    // Enable alarm
    rtc.enableAlarm(rtc.MATCH_DHHMMSS);
  }

  DEBUG_PRINT("Info: "); printDateTime();
  DEBUG_PRINT("Info: Next alarm "); printAlarm();
  alarmFlag = false; // Clear flag
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
  sprintf(dateTimeBuffer, "20%02d-%02d-%02d %02d:%02d:%02d",
          rtc.getYear(), rtc.getMonth(), rtc.getDay(),
          rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
  DEBUG_PRINTLN(dateTimeBuffer);
}

// Print the RTC's alarm
void printAlarm()
{
  char alarmBuffer[25];
  sprintf(alarmBuffer, "20%02d-%02d-%02d %02d:%02d:%02d",
          rtc.getAlarmYear(), rtc.getAlarmMonth(), rtc.getAlarmDay(),
          rtc.getAlarmHours(), rtc.getAlarmMinutes(), rtc.getAlarmSeconds());
  DEBUG_PRINTLN(alarmBuffer);
}
