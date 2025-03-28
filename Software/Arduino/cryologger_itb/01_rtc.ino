// Configure the real-time clock (RTC)
void configureRtc() {
  // Alarm modes:
  // 0: MATCH_OFF          Never
  // 1: MATCH_SS           Every Minute
  // 2: MATCH_MMSS         Every Hour
  // 3: MATCH_HHMMSS       Every Day
  // 4: MATCH_DHHMMSS      Every Month
  // 5: MATCH_MMDDHHMMSS   Every Year
  // 6: MATCH_YYMMDDHHMMSS Once, on a specific date and a specific time

  // Initialize RTC
  rtc.begin();

  // Set time manually
  //rtc.setTime(23, 58, 30); // hours, minutes, seconds
  //rtc.setDate(1, 6, 22); // day, month, year
  //rtc.setEpoch();

  // Set initial RTC alarm time
  rtc.setAlarmTime(0, 0, 0);  // hours, minutes, seconds

  // Enable alarm for hour rollover match
  rtc.enableAlarm(rtc.MATCH_MMSS);
  //rtc.enableAlarm(rtc.MATCH_SS);

  // Attach alarm interrupt service routine (ISR)
  rtc.attachInterrupt(alarmIsr);

  alarmFlag = false;  // Clear flag

  DEBUG_PRINT("Info: RTC initialized ");
  printDateTime();
  DEBUG_PRINT("Info: Initial alarm ");
  printAlarm();
  DEBUG_PRINT("Info: Alarm match ");
  DEBUG_PRINTLN(rtc.MATCH_MMSS);
}

// Read RTC
void readRtc() {
  uint32_t loopStartTime = millis();

  // Get Unix Epoch time
  unixtime = rtc.getEpoch();

  // Write data to union
  moSbdMessage.unixtime = unixtime;

  // Stop loop timer
  timer.readRtc = millis() - loopStartTime;
}

// Set RTC alarm
void setRtcAlarm() {
  // Calculate next alarm
  alarmTime = unixtime + (sampleInterval * 60UL);
  DEBUG_PRINT(F("Info: unixtime "));
  DEBUG_PRINTLN(unixtime);
  DEBUG_PRINT(F("Info: alarmTime "));
  DEBUG_PRINTLN(alarmTime);

  // Check if program is running for first time, alarm is set in the past, or alarm is set too far in the future
  //
  if (firstTimeFlag || (rtc.getEpoch() >= alarmTime) || (alarmTime - unixtime > 86400)) {
    DEBUG_PRINTLN(F("Warning: RTC alarm set in the past or too far in the future."));

    // Set alarm for next hour rollover match
    rtc.setAlarmTime(0, 0, 0);  // hours, minutes, seconds

    // Enable alarm
    rtc.enableAlarm(rtc.MATCH_MMSS);

    DEBUG_PRINT("Info: ");
    printDateTime();
    DEBUG_PRINT("Info: Next alarm ");
    printAlarm();
    DEBUG_PRINT("Info: Alarm match ");
    DEBUG_PRINTLN(rtc.MATCH_MMSS);
  }
  // Check if too many transmission attempt failures have occurred
  else if (failureCounter >= 12) {
    DEBUG_PRINTLN(F("Warning: Increasing RTC alarm interval due to repeated transmission failures"));

    // Set alarm for next day rollover match
    rtc.setAlarmTime(0, 0, 0);  // hours, minutes, seconds

    // Enable alarm for next day rollover match
    rtc.enableAlarm(rtc.MATCH_HHMMSS);

    DEBUG_PRINT("Info: ");
    printDateTime();
    DEBUG_PRINT("Info: Next alarm ");
    printAlarm();
    DEBUG_PRINT("Info: Alarm match ");
    DEBUG_PRINTLN(rtc.MATCH_MMSS);
  } else {
    DEBUG_PRINTLN(F("Info: Setting RTC alarm based on specified interval."));

    // Set alarm time
    rtc.setAlarmTime(hour(alarmTime), minute(alarmTime), 0);  // hours, minutes, seconds

    // Set alarm date
    rtc.setAlarmDate(day(alarmTime), month(alarmTime), year(alarmTime) - 2000);

    // Enable alarm
    rtc.enableAlarm(rtc.MATCH_HHMMSS);

    DEBUG_PRINT("Info: ");
    printDateTime();
    DEBUG_PRINT("Info: Next alarm ");
    printAlarm();
    DEBUG_PRINT("Info: Alarm match ");
    DEBUG_PRINTLN(rtc.MATCH_HHMMSS);
  }
  // Clear flag
  alarmFlag = false;
}

void setCutoffAlarm() {
  // Set alarm for hour rollover match
  rtc.setAlarmTime(0, 0, 0);  // hours, minutes, seconds

  // Enable alarm for hour rollover match
  rtc.enableAlarm(rtc.MATCH_MMSS);

  // Clear flag
  alarmFlag = false;

  DEBUG_PRINT("Info: ");
  printDateTime();
  DEBUG_PRINT("Info: Next alarm ");
  printAlarm();
  DEBUG_PRINT("Info: Alarm match ");
  DEBUG_PRINTLN(rtc.MATCH_HHMMSS);
}

// RTC alarm interrupt service routine (ISR)
void alarmIsr() {
  alarmFlag = true;
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
