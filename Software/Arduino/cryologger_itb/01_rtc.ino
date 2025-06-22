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
  // Start execution timer.
  uint32_t loopStartTime = millis();

  // Get Unix epoch time.
  unixtime = rtc.getEpoch();

  // Write data to the MO-SBD message structure.
  moSbdMessage.unixtime = unixtime;

  // Record elapsed execution time.
  timer.readRtc = millis() - loopStartTime;
}

// ----------------------------------------------------------------------------
// Sets the RTC alarm.
// ----------------------------------------------------------------------------
void setRtcAlarm() {
  // Get current RTC time
  int minute = rtc.getMinutes();
  int hour = rtc.getHours();
  int day = rtc.getDay();
  int month = rtc.getMonth();
  int year = rtc.getYear();  // Offset from 2000 (e.g., 25 = 2025)

  // Align to next hour on first run
  if (firstTimeFlag) {
    DEBUG_PRINTLN("[RTC] Info: First run – aligning to next hour rollover.");
    rtc.setAlarmTime(0, 0, 0);
    rtc.enableAlarm(RTCZero::MATCH_MMSS);
    alarmFlag = false;
    return;
  }

  // Add user-defined intervals
  minute += alarmIntervalMinute;
  hour += alarmIntervalHour;
  day += alarmIntervalDay;

  // Overflow minutes/hours
  if (minute >= 60) {
    hour += minute / 60;
    minute = minute % 60;
  }

  // Overflow hours/days
  if (hour >= 24) {
    day += hour / 24;
    hour = hour % 24;
  }

  // Handle day/month/year overflow
  const byte daysInMonth[12] = {
    31, 28, 31, 30, 31, 30,
    31, 31, 30, 31, 30, 31
  };

  while (true) {
    byte maxDay = daysInMonth[month - 1];
    if (month == 2 && isLeapYear(year)) {
      maxDay = 29;
    }

    if (day <= maxDay) break;

    day -= maxDay;
    month++;
    if (month > 12) {
      month = 1;
      year++;
    }
  }

  // Set alarm
  rtc.setAlarmTime(hour, minute, 0);
  rtc.setAlarmDate(day, month, year);

  // Determine RTC alarm match mode
  RTCZero::Alarm_Match match;
  switch (ALARM_MODE) {
    case MINUTE:
      match = RTCZero::MATCH_MMSS;
      break;
    case HOURLY:
      match = RTCZero::MATCH_HHMMSS;
      break;
    case DAILY:
    default:
      match = RTCZero::MATCH_DHHMMSS;
      break;
  }

  rtc.enableAlarm(match);
  alarmFlag = false;

  DEBUG_PRINT("[RTC] Info: Alarm set for");
  printAlarm();
}

// ----------------------------------------------------------------------------
// Determines if the given RTC year offset from 2000 is a leap year.
// Example: If rtc.year == 24, then it's 2024, which is a leap year.
// ----------------------------------------------------------------------------
bool isLeapYear(int rtcYear) {
  int fullYear = 2000 + rtcYear;
  if ((fullYear % 400) == 0) return true;
  if ((fullYear % 100) == 0) return false;
  if ((fullYear % 4) == 0) return true;
  return false;
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
