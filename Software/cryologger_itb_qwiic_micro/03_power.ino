// Read battery voltage from voltage divider
void readBattery() {

  // Start loop timer
  unsigned long loopStartTime = millis();

  int reading = 0;
  byte samples = 30;

  for (byte i = 0; i < samples; ++i) {
    reading += analogRead(VBAT_PIN); // Read VIN across a 1/10 MΩ resistor divider
    delay(1);
  }

  voltage = (float)reading / samples * 3.3 * ((R2 + R1) / R2) / 4096.0; // Convert 1/10 VIN to VIN (12-bit resolution)

  // Write minimum battery voltage value to union
  if (message.voltage == 0) {
    message.voltage = voltage * 1000;
  }
  else if ((voltage * 1000) < message.voltage) {
    message.voltage = voltage * 1000;
  }

  // print out the value you read:
  //SERIAL_PORT.print(F("voltage: ")); SERIAL_PORT.println(voltage);

  // Stop loop timer
  unsigned long loopEndTime = millis() - loopStartTime;
  //SERIAL_PORT.print(F("readBattery() function execution: ")); SERIAL_PORT.print(loopEndTime); SERIAL_PORT.println(F(" ms"));
}

// Configure the SparkFun Qwiic Power Switch
void configureQwiicPower() {
  if (!mySwitch.begin()) {
    SERIAL_PORT.println(F("Warning: Qwiic Power Switch not detected at default I2C address. Please check wiring."));
  }
}

// Enable power to the SparkFun Qwiic Power Switch
void qwiicPowerOn() {
  mySwitch.powerOn();
}

// Disable power to the SparkFun Qwiic Power Switch
void qwiicPowerOff() {
  mySwitch.powerOff();
}

// Enter deep sleep
void goToSleep() {
#if DEBUG
  SERIAL_PORT.println(F("Entering deep sleep..."));
  SERIAL_PORT.flush();
  SERIAL_PORT.end();
  USBDevice.detach();
#endif
  qwiicPowerOff();      // Disable power
  setPixelColour(off);
  digitalWrite(LED_BUILTIN, LOW);
  LowPower.deepSleep(); // Enter deep sleep

  // Wake up
  wakeUp();
}

// Wake from deep sleep
void wakeUp() {

  if (alarmFlag) {
    readRtc(); // Read RTC
#if DEBUG
    // Re-establish Serial
    USBDevice.attach();
    SERIAL_PORT.begin(115200);
    while (!Serial);
    blinkLed(2, 1000); // Non-blocking delay to allow user to open Serial Monitor
    SERIAL_PORT.println(F(" awake!"));
#endif
    qwiicPowerOn();
    configureNeoPixel();
    configureGnss();        // Configure Sparkfun SAM-M8Q
    configureImu();         // Configure SparkFun ICM-20948
    configureSensors();     // Configure attached sensors
    //configureIridium();     // Configure SparkFun Qwiic Iridium 9603N
  }
}

// Blink LED (non-blocking)
void blinkLed(byte ledFlashes, unsigned int ledDelay) {

  pinMode(LED_BUILTIN, OUTPUT);
  byte i = 0;

  while (i < ledFlashes * 2) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis > ledDelay) {
      previousMillis = currentMillis;
      ledState = !ledState;
      digitalWrite(LED_BUILTIN, ledState);
      i++;
    }
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void blinkLED(unsigned long interval) {
  digitalWrite(LED_BUILTIN, (millis() / interval) % 2 == 1 ? HIGH : LOW);
}
