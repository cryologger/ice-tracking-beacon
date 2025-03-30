/*
  RTC Module

  This module configures and manages the SAMD21 real-time clock (RTC).
  It sets alarms, manages alarm interrupt handling, and provides
  utility functions to read and print RTC time and alarms. 
  
  -----------------------------------------------------------------------------
  Alarm Modes:
  -----------------------------------------------------------------------------
  0: MATCH_OFF            Disabled
  1: MATCH_SS             Every minute
  2: MATCH_MMSS           Every hour
  3: MATCH_HHMMSS         Every day
  4: MATCH_DHHMMSS        Every month
  5: MATCH_MMDDHHMMSS     Every year
  6: MATCH_YYMMDDHHMMSS   Once, on a specific date and a specific time
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
  //rtc.setEpoch();          // sets time to specified epoch

  // Set the initial alarm time (HH:MM:SS).
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
  DEBUG_PRINT("[RTC] Info: Initial alarm ");
  printAlarm();
  DEBUG_PRINT("[RTC] Info: Alarm match ");
  DEBUG_PRINTLN(rtc.MATCH_MMSS);
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
  // Calculate next alarm epoch.
  alarmTime = unixtime + (transmitInterval * 3600UL);
  DEBUG_PRINT(F("Info: unixtime "));
  DEBUG_PRINTLN(unixtime);
  DEBUG_PRINT(F("Info: alarmTime "));
  DEBUG_PRINTLN(alarmTime);

  // If the program is running for the first time, or if the alarmTime is
  // invalid, set alarm for next hour rollover match.
  if (firstTimeFlag
      || (rtc.getEpoch() >= alarmTime)
      || ((alarmTime - unixtime) > 86400)) {
    DEBUG_PRINTLN(F("Warning: RTC alarm in the past or too far in future."));

    // Set alarm for next hour rollover match.
    rtc.setAlarmTime(0, 0, 0);
    rtc.enableAlarm(rtc.MATCH_MMSS);

    DEBUG_PRINT("[RTC] Info: ");
    printDateTime();
    DEBUG_PRINT("[RTC] Info: Next alarm ");
    printAlarm();
    DEBUG_PRINT("[RTC] Info: Alarm match ");
    DEBUG_PRINTLN(rtc.MATCH_MMSS);
  }
  // If there are repeated failures, increase the alarm interval to daily.
  else if (failureCounter >= 12) {
    DEBUG_PRINTLN(F("[RTC] Warning: Increasing alarm interval due to transmission failures."));

    // Set alarm for next day rollover match.
    rtc.setAlarmTime(0, 0, 0);
    rtc.enableAlarm(rtc.MATCH_HHMMSS);

    DEBUG_PRINT("[RTC] Info: ");
    printDateTime();
    DEBUG_PRINT("[RTC] Info: Next alarm ");
    printAlarm();
    DEBUG_PRINT("[RTC] Info: Alarm match ");
    DEBUG_PRINTLN(rtc.MATCH_MMSS);
  } else {
    // Otherwise, set alarm time based on transmitInterval hours.
    DEBUG_PRINTLN(F("[RTC] Info: Setting RTC alarm based on interval."));

    rtc.setAlarmTime(hour(alarmTime), minute(alarmTime), 0);
    rtc.setAlarmDate(day(alarmTime), month(alarmTime),
                     year(alarmTime) - 2000);
    rtc.enableAlarm(rtc.MATCH_HHMMSS);

    DEBUG_PRINT("[RTC] Info: ");
    printDateTime();
    DEBUG_PRINT("[RTC] Info: Next alarm ");
    printAlarm();
    DEBUG_PRINT("[RTC] Info: Alarm match ");
    DEBUG_PRINTLN(rtc.MATCH_HHMMSS);
  }

  alarmFlag = false;  // Clear flag
}

// ----------------------------------------------------------------------------
// Sets the alarm for the next hour rollover match when battery voltage is
// below cutoff.
// ----------------------------------------------------------------------------
void setCutoffAlarm() {
  // Set alarm for hour rollover match.
  rtc.setAlarmTime(0, 0, 0);
  rtc.enableAlarm(rtc.MATCH_MMSS);

  alarmFlag = false;  // Clear flag

  DEBUG_PRINT("[RTC] Info: ");
  printDateTime();
  DEBUG_PRINT("[RTC] Info: Next alarm ");
  printAlarm();
  DEBUG_PRINT("[RTC] Info: Alarm match ");
  DEBUG_PRINTLN(rtc.MATCH_HHMMSS);
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
// Print the RTC's alarm time in "YYYY-MM-DD HH:MM:SS" format.
// ----------------------------------------------------------------------------
void printAlarm() {
  char alarmBuffer[30];
  snprintf(alarmBuffer, sizeof(alarmBuffer),
           "20%02d-%02d-%02d %02d:%02d:%02d",
           rtc.getAlarmYear(), rtc.getAlarmMonth(), rtc.getAlarmDay(),
           rtc.getAlarmHours(), rtc.getAlarmMinutes(), rtc.getAlarmSeconds());
  DEBUG_PRINTLN(alarmBuffer);
}
