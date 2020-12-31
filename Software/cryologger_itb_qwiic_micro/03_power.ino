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

  //DEBUG_PRINT("voltage: "); DEBUG_PRINTLN(voltage);

  // Stop loop timer
  unsigned long loopEndTime = millis() - loopStartTime;
  //DEBUG_PRINT("readBattery() function execution: "); DEBUG_PRINT(loopEndTime); DEBUG_PRINTLN(" ms");
}

// Disable serial port
void disableSerial()
{
#if DEBUG
  SERIAL_PORT.flush();  // Wait for transmission of any serial data to complete
  SERIAL_PORT.end();    // Close serial port
  USBDevice.detach();   // Safely detach USB prior to sleeping
#endif
}

// Enable serial port
void enableSerial()
{
#if DEBUG
  USBDevice.attach(); // Re-attach USB
  SERIAL_PORT.begin(115200);
  //while (!SERIAL_PORT);
  blinkLed(2, 1000); // Non-blocking delay to allow user to open Serial Monitor
#endif
}

// Enable power to MOSFET
void enablePower() {
  digitalWrite(PIN_MOSFET, LOW);
}

// Disable power to MOSFET
void disablePower() {
  digitalWrite(PIN_MOSFET, HIGH);
}

// Enter deep sleep
void goToSleep() {

  DEBUG_PRINTLN("Entering deep sleep...");
  DEBUG_PRINTLN();

  // Do not change serial settings on Watchdog Timer interrupts
  if (!watchdogFlag)
  {
    disableSerial();
  }

  // Clear first-time flag after initial power-down
  if (firstTimeFlag)
  {
    firstTimeFlag = false;
  }

  setLedColour(off); // Turn off LED
  digitalWrite(LED_BUILTIN, LOW);
  disablePower();

  // Enter deep sleep
  LowPower.deepSleep();

  /* Code sleeps here and awaits RTC or WDT interrupt */
}

// Wake from deep sleep
void wakeUp() {

#if DEBUG
  // Re-configure all devices
  enableSerial();       // Re-enable serial port
#endif

  enablePower();
  configureLed();       // Configure WS2812B RGB LED
  configureGnss();      // Configure Sparkfun SAM-M8Q
  configureImu();       // Configure SparkFun ICM-20948
  configureSensors();   // Configure attached sensors
  configureIridium();   // Configure Iridium 9603
}

// Blink LED (non-blocking) (https://forum.arduino.cc/index.php?topic=503368.0)
void blinkLed(byte ledFlashes, unsigned int ledDelay)
{
  byte i = 0;
  while (i < ledFlashes * 2)
  {
    currentMillis = millis();
    if (currentMillis - previousMillis >= ledDelay)
    {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      previousMillis = currentMillis;
      i++;
    }
  }
  digitalWrite(LED_BUILTIN, LOW);
}
