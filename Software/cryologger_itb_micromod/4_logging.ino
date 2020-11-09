void configureSd() {
  if (sd.begin(PIN_MICROSD_CHIP_SELECT, SD_SCK_MHZ(24))) {
    online.microSd = true;
  }
  else {
    Serial.println(F("Warning: microSD not detected! Please check wiring."));
    //while(1);
  }
}

void enableLogging() {

  // Get the RTC's current date and time
  rtc.getTime();

  // Create log file name
  sprintf(fileName, "20%02d%02d%02d_%02d0000.csv",
          rtc.year, rtc.month, rtc.dayOfMonth, rtc.hour);
  //rtc.hour, rtc.minute, rtc.seconds

  // O_CREAT - Create the file if it does not exist
  // O_APPEND - Seek to the end of the file prior to each write
  // O_WRITE - Open the file for writing
  if (!file.open(fileName, O_CREAT | O_APPEND | O_WRITE)) {
    Serial.println(F("Failed to create log file"));
    return;
  }

  Serial.print(F("Logging to file: ")); Serial.println(fileName);

  updateDataFileCreate(); // Update the file creation timestamp

}

void updateDataFileCreate() {
  // Get the RTC's current date and time
  rtc.getTime();
  // Update the file create timestamp
  if (!file.timestamp(T_CREATE, (rtc.year + 2000), rtc.month, rtc.dayOfMonth, rtc.hour, rtc.minute, rtc.seconds)) {
    Serial.print(F("Warning: Unable to write file create timestamp"));
  }
}

void updateDataFileAccess() {
  // Get the RTC's current date and time
  rtc.getTime();
  // Update the file access timestamp
  if (!file.timestamp(T_ACCESS, (rtc.year + 2000), rtc.month, rtc.dayOfMonth, rtc.hour, rtc.minute, rtc.seconds)) {
    Serial.print(F("Warning: Unable to write file access timestamp"));
  }
  // Update the file write timestamp
  if (!file.timestamp(T_WRITE, (rtc.year + 2000), rtc.month, rtc.dayOfMonth, rtc.hour, rtc.minute, rtc.seconds)) {
    Serial.print(F("Warning: Unable to write file write timestamp"));
  }
}
