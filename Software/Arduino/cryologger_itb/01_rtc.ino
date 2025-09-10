/*
  RTC Module

  This module configures and manages the SAMD21 real-time clock (RTC).
  It sets alarms, manages alarm interrupt handling, and provides
  utility functions to read and print RTC time and alarms. 
  
  -----------------------------------------------------------------------------
  Alarm Modes:
  -----------------------------------------------------------------------------
  MATCH_OFF            Disabled
  MATCH_SS             Every minute
  MATCH_MMSS           Every hour
  MATCH_HHMMSS         Every day
  MATCH_DHHMMSS        Every month
  MATCH_MMDDHHMMSS     Every year
  MATCH_YYMMDDHHMMSS   Once, on a specific date and time
*/

// ----------------------------------------------------------------------------
// Initializes the RTC and optionally sets the date/time for debugging.
// Attaches an interrupt handler for the RTC alarm ISR.
// ----------------------------------------------------------------------------
void configureRtc() {

  // Initialize the RTC.
  rtc.begin();

  // Optional manual time setting:
  //rtc.setTime(23, 58, 30); // hours, minutes, seconds
  //rtc.setDate(1, 6, 22);   // day, month, year
  //rtc.setEpoch();          // Sets the time to specified epoch

  // Set the initial alarm time to the next hour rollover (hours, minutes, seconds).
  rtc.setAlarmTime(0, 0, 0);

  // Enable alarm for hour rollover match.
  rtc.enableAlarm(rtc.MATCH_MMSS);
  // rtc.enableAlarm(rtc.MATCH_SS);

  // Attach alarm interrupt service routine (ISR).
  rtc.attachInterrupt(alarmIsr);

  // Clear the alarm flag.
  alarmFlag = false;

  DEBUG_PRINT("[RTC] Info: RTC initialized ");
  printDateTime();
}

// ----------------------------------------------------------------------------
// Reads the current epoch time from the RTC.
// ----------------------------------------------------------------------------
void readRtc() {
  // Start execution timer
  uint32_t loopStartTime = millis();

  // Get Unix epoch time
  unixtime = rtc.getEpoch();

  // Write data to the MO-SBD message structure
  moSbdMessage.unixtime = unixtime;

  // Record elapsed execution time
  timer.readRtc = millis() - loopStartTime;
}

// ----------------------------------------------------------------------------
// Sets the RTC alarm.
// ----------------------------------------------------------------------------
void setRtcAlarm() {
  // On first boot, align to next hour boundary using MATCH_MMSS
  if (firstTimeFlag) {
    DEBUG_PRINTLN("[RTC] Info: First run – aligning to next hour rollover.");
    rtc.setAlarmTime(0, 0, 0);
    rtc.enableAlarm(RTCZero::MATCH_MMSS);
    alarmFlag = false;
    return;
  }

  // Read current time from RTC
  uint32_t current_epoch = rtc.getEpoch();

  // Compute total interval in seconds
  uint32_t interval_sec =
    alarmIntervalMinute * 60UL
    + alarmIntervalHour * 3600UL
    + alarmIntervalDay * 86400UL;

  // Safety check — ensure we always have a valid interval
  if (interval_sec == 0) {
    DEBUG_PRINTLN("[RTC] Warning: Alarm interval is 0! Falling back to default 1 hour.");
    interval_sec = 3600UL;  // Default to 1 hour
  }

  // Align to the next clean interval boundary
  uint32_t next_epoch = ((current_epoch + interval_sec) / interval_sec) * interval_sec;

  // Convert to date/time
  tmElements_t tm;
  breakTime(next_epoch, tm);  // Handles all calendar overflow and leap year logic

  int second = tm.Second;
  int minute = tm.Minute;
  int hour = tm.Hour;
  int day = tm.Day;
  int month = tm.Month;
  int year = (tm.Year + 1970) - 2000;  // RTC year offset from 2000 (tm.Year is years since 1970)

  // Set alarm time and date
  rtc.setAlarmTime(hour, minute, second);
  rtc.setAlarmDate(day, month, year);

  // Determine match mode based on user config
  RTCZero::Alarm_Match match;
  switch (alarmMode) {
    case MINUTE:
      match = RTCZero::MATCH_HHMMSS;  // Matches every HH:MM:SS for sub-hour intervals
      break;
    case HOURLY:
      match = RTCZero::MATCH_HHMMSS;  // Matches every HH:MM:SS
      break;
    case DAILY:
    default:
      match = RTCZero::MATCH_DHHMMSS;  // Matches every DD:HH:MM:SS
      break;
  }

  rtc.enableAlarm(match);
  alarmFlag = false;

  DEBUG_PRINT("[RTC] Info: Alarm set for ");
  printAlarm();
}

// ----------------------------------------------------------------------------
// RTC alarm interrupt service routine (ISR).
// ----------------------------------------------------------------------------
void alarmIsr() {
  alarmFlag = true;
}

// ----------------------------------------------------------------------------
// Prints the RTC's current date and time in "YYYY-MM-DD HH:MM:SS" format.
// ----------------------------------------------------------------------------
void printDateTime() {
  char dateTimeBuffer[30];
  snprintf(dateTimeBuffer, sizeof(dateTimeBuffer),
           "20%02d-%02d-%02d %02d:%02d:%02d",
           rtc.getYear(), rtc.getMonth(), rtc.getDay(),
           rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
  DEBUG_PRINTLN(dateTimeBuffer);
}

// ----------------------------------------------------------------------------
// Prints the RTC's alarm time in "YYYY-MM-DD HH:MM:SS" format.
// ----------------------------------------------------------------------------
void printAlarm() {
  char alarmBuffer[30];
  snprintf(alarmBuffer, sizeof(alarmBuffer),
           "20%02d-%02d-%02d %02d:%02d:%02d",
           rtc.getAlarmYear(), rtc.getAlarmMonth(), rtc.getAlarmDay(),
           rtc.getAlarmHours(), rtc.getAlarmMinutes(), rtc.getAlarmSeconds());
  DEBUG_PRINTLN(alarmBuffer);
}

// ----------------------------------------------------------------------------
// Prints a UNIX timestamp in human-readable "YYYY-MM-DD HH:MM:SS" format.
// ----------------------------------------------------------------------------
void printUnixtime(time_t epoch) {
  char dateTimeBuffer[30];
  tmElements_t tm;
  breakTime(epoch, tm);  // Convert epoch to tm structure
  snprintf(dateTimeBuffer, sizeof(dateTimeBuffer),
           "%04d-%02d-%02d %02d:%02d:%02d",
           tm.Year + 1970, tm.Month, tm.Day,
           tm.Hour, tm.Minute, tm.Second);

  DEBUG_PRINTLN(dateTimeBuffer);
}
