// Configure the SparkFun Qwiic Power Switch
void configureQwiicPower() {
  if (!mySwitch.begin()) {
    Serial.println(F("Warning: Qwiic Power Switch not detected at default I2C address. Please check wiring."));
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
  Serial.println(F("Entering deep sleep..."));
  Serial.flush();
  Serial.end();
  USBDevice.detach();
#endif
  qwiicPowerOff();      // Disable power
  LowPower.deepSleep(); // Enter deep sleep

  // Wake up
  wakeUp();
}

// Wake from deep sleep
void wakeUp() {

  // Re-establish Serial upon alarm trigger
  if (alarmFlag) {
#if DEBUG
    USBDevice.attach();
    //while (!Serial);
    delay(5000);
    Serial.begin(115200);
    qwiicPowerOn();
    configureNeoPixel();
    configureGnss();        // Configure Sparkfun SAM-M8Q
    configureImu();         // Configure SparkFun ICM-20948
    configureSensors();
    configureIridium();     // Configure SparkFun Qwiic Iridium 9603N
#endif
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
