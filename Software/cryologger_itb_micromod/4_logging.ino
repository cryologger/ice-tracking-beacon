void configureSd() {
  if (sd.begin(PIN_MICROSD_CHIP_SELECT, SD_SCK_MHZ(24))) {

  }
  else {
    Serial.println(F("Warning: microSD not detected! Please check wiring."));
    //while(1);
  }

}
