// Read battery voltage from voltage divider
void readBattery() {

  // Start loop timer
  unsigned long loopStartTime = millis();

  int reading = 0;
  byte samples = 30;

  for (byte i = 0; i < samples; ++i) {
    reading += analogRead(PIN_VBAT); // Read VIN across a 1/10 MΩ resistor divider
    delay(1);
  }

  voltage = (float)reading / samples * 3.3 * ((R2 + R1) / R2) / 4096.0; // Convert 1/10 VIN to VIN (12-bit resolution)

  // Write minimum battery voltage value to union
  if (moMessage.voltage == 0) {
    moMessage.voltage = voltage * 1000;
  }
  else if ((voltage * 1000) < moMessage.voltage) {
    moMessage.voltage = voltage * 1000;
  }

  //DEBUG_PRINT("voltage: "); SERIAL_PORT.println(voltage);

  // Stop loop timer
  unsigned long loopEndTime = millis() - loopStartTime;
  //DEBUG_PRINT("readBattery() function execution: "); DEBUG_PRINT(loopEndTime); DEBUG_PRINTLN(" ms");
}

// Disable serial port
void disableSerial() {

  // Wait for transmission of any serial data to complete
  SERIAL_PORT.flush();

  // Close serial port
  SERIAL_PORT.end();

  // Safely detach USB prior to sleeping
  USBDevice.detach();
}

// Enable serial port
void enableSerial() {
  USBDevice.attach(); // Re-attach USB
  SERIAL_PORT.begin(115200);
  //while (!SERIAL_PORT);
  blinkLed(2, 1000); // Non-blocking delay to allow user to open Serial Monitor
}

// Enter deep sleep
void goToSleep() {

  if (firstTimeFlag) {
    disableSerial();
    firstTimeFlag = false;
  }
  DEBUG_PRINTLN("Entering deep sleep...");
  DEBUG_PRINTLN();

  setLedColour(off);
  LowPower.deepSleep(); // Enter deep sleep
}

// Wake from deep sleep
void wakeUp() {

  // Re-configure all devices
  enableSerial();       // Re-enable serial port
  configureLed();       // Configure WS2812B RGB LED
  configureGnss();      // Configure Sparkfun SAM-M8Q
  configureImu();       // Configure SparkFun ICM-20948
  configureSensors();   // Configure attached sensors
  configureIridium();   // Configure SparkFun Qwiic Iridium 9603N
}

// Blink LED (non-blocking)
void blinkLed(byte ledFlashes, unsigned int ledDelay) {

  pinMode(LED_BUILTIN, OUTPUT);
  byte i = 0;

  while (i < ledFlashes * 2) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis > ledDelay) {
      previousMillis = currentMillis;
      ledStateFlag = !ledStateFlag;
      digitalWrite(LED_BUILTIN, ledStateFlag);
      i++;
    }
  }
  digitalWrite(LED_BUILTIN, LOW);
}
