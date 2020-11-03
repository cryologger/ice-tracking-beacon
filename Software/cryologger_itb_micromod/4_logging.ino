void configureSd() {
  if (sd.begin(PIN_MICROSD_CHIP_SELECT, SD_SCK_MHZ(24))) {

  }
  else {
    Serial.println(F("Warning: microSD not detected! Please check wiring."));
    //while(1);
  }

}



void updateDataFileCreate(SdFile *file) {
  rtc.getTime(); 
  // Update the file create timestamp
  file->timestamp(T_CREATE, (rtc.year + 2000), rtc.month, rtc.dayOfMonth, rtc.hour, rtc.minute, rtc.seconds);
}

void updateDataFileAccess(SdFile *file) {
  rtc.getTime(); 
  // Update the file access timestamp
  file->timestamp(T_ACCESS, (rtc.year + 2000), rtc.month, rtc.dayOfMonth, rtc.hour, rtc.minute, rtc.seconds);
  // Update the file write timestamp
  file->timestamp(T_WRITE, (rtc.year + 2000), rtc.month, rtc.dayOfMonth, rtc.hour, rtc.minute, rtc.seconds);
}
