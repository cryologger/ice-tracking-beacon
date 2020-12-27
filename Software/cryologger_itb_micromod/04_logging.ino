void configureSd() {

  if (sd.begin(PIN_SD_CHIP_SELECT, SD_SCK_MHZ(24))) {
    online.microSd = true;
  }
  else {
    DEBUG_PRINTLN("Warning: microSD not detected! Please check wiring.");
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
  sprintf(fileName, "20%02d%02d%02d_%02d%02d%02d.csv",
          rtc.year, rtc.month, rtc.dayOfMonth,
          rtc.hour, rtc.minute, rtc.seconds);

  // O_CREAT - Create the file if it does not exist
  // O_APPEND - Seek to the end of the file prior to each write
  // O_WRITE - Open the file for writing
  if (!file.open(fileName, O_CREAT | O_APPEND | O_WRITE)) {
    DEBUG_PRINTLN("Failed to create log file");
    return;
  }

  if (!file.isOpen()) {
    DEBUG_PRINTLN("Warning: Unable to open file");
  }

  // Update file create timestamp
  updateFileCreate();

  // Write header to file
  file.println("unixtime,temperature,humidity,pressure,latitude,longitude,satellites,pdop,drift,voltage,transmitDuration,messageCounter");

  // Sync the log file
  file.sync();

  // Close log file
  file.close();

  DEBUG_PRINT("Logging to file: "); DEBUG_PRINTLN(fileName);
}


// Log data to microSD
void logData() {

  // Open log file and append data
  if (file.open(fileName, O_APPEND | O_WRITE)) {
    file.print(moMessage.unixtime);             file.print(",");
    file.print(moMessage.temperature / 100.0);  file.print(",");
    file.print(moMessage.humidity / 100.0);     file.print(",");
    file.print(moMessage.pressure / 100.0);     file.print(",");
    file.print(moMessage.latitude);             file.print(",");
    file.print(moMessage.longitude);            file.print(",");
    file.print(moMessage.satellites);           file.print(",");
    file.print(moMessage.pdop);                 file.print(",");
    file.print(moMessage.rtcDrift);             file.print(",");
    file.print(moMessage.voltage, 2);           file.print(",");
    file.print(moMessage.transmitDuration);     file.print(",");
    file.println(moMessage.messageCounter);

    updateFileAccess(); // Update file access and write timestamps
  }
  else {
    DEBUG_PRINTLN("Warning: Unable to open file!");
  }

  // Sync log file
  if (!file.sync()) {
    DEBUG_PRINTLN(F("Warning: File sync error!"));
  }

  // Check for write error
  if (file.getWriteError()) {
    DEBUG_PRINTLN(F("Warning: File write error!"));
  }

  // Close log file
  if (!file.close()) {
    DEBUG_PRINTLN(F("Warning: File close error!"));
  }

  // Blink LED
  blinkLed(2, 100);
}

// Update the file create timestamp
void updateFileCreate() {
  // Get the RTC's current date and time
  rtc.getTime();
  // Update the file create timestamp
  if (!file.timestamp(T_CREATE, (rtc.year + 2000), rtc.month, rtc.dayOfMonth,
                      rtc.hour, rtc.minute, rtc.seconds)) {
    DEBUG_PRINTLN("Warning: Unable to write file create timestamp");
  }
}

// Update the file access and write timestamps
void updateFileAccess() {
  // Get the RTC's current date and time
  rtc.getTime();
  // Update the file access timestamp
  if (!file.timestamp(T_ACCESS, (rtc.year + 2000), rtc.month, rtc.dayOfMonth,
                      rtc.hour, rtc.minute, rtc.seconds)) {
    DEBUG_PRINTLN("Warning: Unable to write file access timestamp");
  }
  // Update the file write timestamp
  if (!file.timestamp(T_WRITE, (rtc.year + 2000), rtc.month, rtc.dayOfMonth,
                      rtc.hour, rtc.minute, rtc.seconds)) {
    DEBUG_PRINTLN("Warning: Unable to write file write timestamp");
  }
}
