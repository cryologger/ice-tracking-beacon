void configureSd() {
  if (sd.begin(PIN_MICROSD_CHIP_SELECT, SD_SCK_MHZ(24))) {
    online.microSd = true;
  }
  else {
    Serial.println(F("Warning: microSD not detected! Please check wiring."));
    //while(1);
  }
}

// Create log file
void createLogFile() {

  if (file.isOpen())
    file.close();

  // Get the RTC's current date and time
  rtc.getTime();

  // Create log file name
  sprintf(fileName, "20%02d%02d%02d_%02d0000.csv",
          rtc.year, rtc.month, rtc.dayOfMonth, rtc.hour);

  // O_CREAT - Create the file if it does not exist
  // O_APPEND - Seek to the end of the file prior to each write
  // O_WRITE - Open the file for writing
  if (!file.open(fileName, O_CREAT | O_APPEND | O_WRITE)) {
    Serial.println(F("Failed to create log file"));
    return;
  }

  if (!file.isOpen()) {
    Serial.println(F("Warning: Unable to open file"));
  }

  // Update file create timestamp
  updateDataFileCreate();

  // Write header to file
  file.println("datetime,latitude,longitude,sattlites,fix,pdop,");

  // Sync the log file
  file.sync();

  // Close log file
  file.close();

  Serial.print(F("Logging to file: ")); Serial.println(fileName);
}

void logData() {

  // Open log file and append data
  if (file.open(fileName, O_APPEND | O_WRITE)) {
    file.write(outputData, strlen(outputData)); // Write data to SD
    updateDataFileAccess(); // Update file access and write timestamps
  }
  else {
    Serial.println("Warning: Unable to open file");
  }

  // Force data to SD and update the directory entry to avoid data loss
  if (!file.sync() || file.getWriteError()) {
    Serial.println(F("Warning: Write error"));
  }

  file.close();
  
  // Print outputData to terminal
  Serial.print("outputData: "); Serial.println(outputData);

  // Clear arrays
  memset(outputData, 0x00, sizeof(outputData));
  memset(tempData, 0x00, sizeof(tempData)); 
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
    Serial.println(F("Warning: Unable to write file access timestamp"));
  }
  // Update the file write timestamp
  if (!file.timestamp(T_WRITE, (rtc.year + 2000), rtc.month, rtc.dayOfMonth, rtc.hour, rtc.minute, rtc.seconds)) {
    Serial.println(F("Warning: Unable to write file write timestamp"));
  }
}
