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
void enablePower()
{
  digitalWrite(PIN_MOSFET, LOW);
  myDelay(2000);  // Non-blocking delay
}

// Disable power to MOSFET
void disablePower()
{
  digitalWrite(PIN_MOSFET, HIGH);
}

// Enter deep sleep
void goToSleep()
{
  // Clear first-time flag after initial power-down
  if (firstTimeFlag)
  {
    firstTimeFlag = false;
  }

  // Turn off LEDs
  setLedColour(CRGB::Black);
  digitalWrite(LED_BUILTIN, LOW);

  // Disable power
  //disablePower();

  // Enter deep sleep
  LowPower.deepSleep();

  /* Code sleeps here and awaits RTC or WDT interrupt */
}

// Wake from deep sleep
void wakeUp()
{
  // Re-configure devices

#if DEBUG
  enableSerial();       // Re-enable serial port
#endif
  //enablePower();        // Enable power to MOSFET controlled components
  configureLed();       // Configure WS2812B RGB LED
  configureGnss();      // Configure GNSS receiver
  configureImu();       // Configure interial measurement unit
  configureSensors();   // Configure attached sensors
  //configureIridium();   // Configure Iridium 9603 modem
}

// Non-blocking blink LED (https://forum.arduino.cc/index.php?topic=503368.0)
void blinkLed(byte ledFlashes, unsigned int ledDelay)
{
  byte i = 0;
  while (i < ledFlashes * 2)
  {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= ledDelay)
    {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      previousMillis = currentMillis;
      i++;
    }
  }
  // Ensure LED is off at end of blink cycle
  digitalWrite(LED_BUILTIN, LOW);
}

// Non-blocking delay (ms: duration)
// https://arduino.stackexchange.com/questions/12587/how-can-i-handle-the-millis-rollover
void myDelay(unsigned long ms)
{
  unsigned long start = millis();         // Start: timestamp
  for (;;)
  {
    petDog();                             // Reset watchdog timer
    unsigned long now = millis();         // Now: timestamp
    unsigned long elapsed = now - start;  // Elapsed: duration
    if (elapsed >= ms)                    // Comparing durations: OK
      return;
  }
}
