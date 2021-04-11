// Configure real-time clock
void configureRtc()
{
  // Initialize RTC
  if (!rtc.begin())
  {
    DEBUG_PRINTLN("Warning: RTC initialization failed!");
  }
  else
  {
    online.rtc; // Set flag
  }

  // Enable 24-hour time format
  rtc.set24Hour();

  DEBUG_PRINT("Info: Time at power up "); printDateTime();
}


void setInitialAlarm()
{
  // Update time variables from RTC
  rtc.updateTime();

  // Disable all RTC interrupts
  rtc.disableAllInterrupts();

  // Ensure all interrupt flags are cleared
  rtc.clearAllInterruptFlags();

  // Select alarm interrupt registers to compare with the current time registers
  rtc.setItemsToMatchForAlarm(1, 0, 0, 0); // minutes, hours, weekday, date

  // Set initial alarm to occur on hour rollover
  //rtc.setAlarmMinutes(0);

  // Set initial alarm
  rtc.setAlarmMinutes((rtc.getMinutes() + alarmMinutes) % 60);

  // Generate an interrupt signal on the INT pin when an alarm match occurs
  rtc.enableHardwareInterrupt(ALARM_INTERRUPT);

  // Configure and attach interrupt on the INT pin
  pinMode(PIN_RTC_INT, INPUT_PULLUP);
  LowPower.attachInterruptWakeup(PIN_RTC_INT, alarmIsr, FALLING);
}

// Read RTC
void readRtc()
{
  // Start loop timer
  unsigned long loopStartTime = micros();

  // Update time variables from RTC
  if (!rtc.updateTime())
  {
    DEBUG_PRINT("Warning: RTC failed to update!");
    setLedColour(CRGB::Orange);
  }

  // Get UNIX Epoch time
  unixtime = rtc.getEpoch();

  // Write data to union
  moMessage.unixtime = unixtime;

  //DEBUG_PRINT("Epoch time: "); DEBUG_PRINTLN(unixtime);

  // Stop the loop timer
  timer.rtc = micros() - loopStartTime;
}

// Set RTC alarm time and date
void setRtcAlarm()
{
  // If multiple failed transmissions are detected, the RTC alarm 
  // interval will increase up to a maximum of 1 day.
  // When a transmission is made successfully, the RTC alarm 
  // interval will return to the default value
  
  if (failedTransmitCounter < 5)
  {
    // Calculate next alarm
    alarmTime = unixtime + alarmInterval;
  }
  else if (failedTransmitCounter > 5 && failedTransmitCounter < 5 )
  {
    // Calculate next alarm
    alarmTime = unixtime + 43200;
  }
  else
  {
    // Calculate next alarm
    alarmTime = unixtime + 86400;
  }


  // Update time variables from RTC
  rtc.updateTime();

  // Check if the alarm was set in the past
  if (alarmTime < rtc.getEpoch())
  {
    unixtime = rtc.getEpoch(); // Get UNIX Epoch time
    alarmTime = unixtime + alarmInterval; // Recalculate next alarm
    DEBUG_PRINTLN("Warning: RTC alarm set in the past!");

    // Select alarm interrupt registers to compare with the current time registers
    rtc.setItemsToMatchForAlarm(1, 0, 0, 0); // minutes, hours, weekday, date

    // Set alarm to occur on next hour rollover
    rtc.setAlarmMinutes(0);
  }
  else
  {
    // Check if the program is running for the first time
    if (firstTimeFlag)
    {
      DEBUG_PRINTLN("Setting initial RTC alarm...");

      // Select alarm interrupt registers to compare with the current time registers
      rtc.setItemsToMatchForAlarm(1, 0, 0, 0); // minutes, hours, weekday, date

      // Set initial alarm to occur on next hour rollover
      //rtc.setAlarmMinutes(0);
      rtc.setAlarmMinutes((rtc.getMinutes() + alarmMinutes) % 60);
    }
    else
    {
      // Select alarm interrupt registers to compare with the current time registers
      rtc.setItemsToMatchForAlarm(1, 1, 0, 0); // minutes, hours, weekday, date

      // Set RTC alarm
      rtc.setAlarmMinutes(minute(alarmTime));
      rtc.setAlarmHours(hour(alarmTime));
      rtc.setAlarmDate(day(alarmTime));
    }
  }

  // Set RTC rolling alarm
  //rtc.setAlarmMinutes((rtc.getMinutes() + alarmMinutes) % 60);
  //rtc.setAlarmHours((rtc.getHours() + alarmHours) % 24);
  //rtc.setAlarmWeekday(0);
  //rtc.setAlarmDate((rtc.getDate() + alarmDate) % 31);

  // Clear RTC alarm interrupt flag
  rtc.clearInterruptFlag(FLAG_ALARM);

  // Print the next RTC alarm date and time
  DEBUG_PRINT("Current time: "); printDateTime();
  DEBUG_PRINT("Next alarm: "); printAlarm();
}

// RTC alarm interrupt service routine (ISR)
void alarmIsr()
{
  alarmFlag = true; // Set alarm flag
}


// Print the RTC date and time of last call to .updateTime()
void printDateTime()
{
  char dateTimeBuffer[25];
  sprintf(dateTimeBuffer, "%04d-%02d-%02d %02d:%02d:%02d",
          rtc.getYear(), rtc.getMonth(), rtc.getDate(),
          rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
  DEBUG_PRINTLN(dateTimeBuffer);
}

// Print the RTC's current date and time
void printCurrentDateTime()
{
  rtc.updateTime();
  char dateTimeBuffer[25];
  sprintf(dateTimeBuffer, "%04d-%02d-%02d %02d:%02d:%02d",
          rtc.getYear(), rtc.getMonth(), rtc.getDate(),
          rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
  DEBUG_PRINTLN(dateTimeBuffer);
}

// Print the RTC's alarm
void printAlarm()
{
  char alarmBuffer[25];
  sprintf(alarmBuffer, "%04d-%02d-%02d %02d:%02d:00",
          rtc.getYear(), rtc.getMonth(), rtc.getAlarmDate(),
          rtc.getAlarmHours(), rtc.getAlarmMinutes());
  DEBUG_PRINTLN(alarmBuffer);
}
