void configurePower() {
  if (!mySwitch.begin()) {
    Serial.println(F("Warning: Qwiic Power Switch not detected at default I2C address. Please check wiring."));
  }
}

void qwiicPowerOn() {
  mySwitch.powerOn();
}

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

  // Reestablish Serial if alarm trigger
  if (alarmFlag) {
#if DEBUG
    USBDevice.attach();
    while (!Serial);
    delay(5000);
    Serial.begin(115200);
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
