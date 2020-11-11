// Enter deep sleep
void goToSleep() {


  mySwitch.powerOff();  // Disable power
  LowPower.deepSleep(); // Enter deep sleep

  // Wake up
  wakeUp();
}

// Wake from deep sleep
void wakeUp() {

  // If alarm is triggered, enable power to system
  if (alarmFlag) {

    
  }

}


// Blink LED (non-blocking)
void blinkLed(byte ledFlashes, unsigned int ledDelay) {

  pinMode(LED_BUILTIN, OUTPUT);
  byte i = 0;

  while (i <= ledFlashes * 2) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= ledDelay) {
      previousMillis = currentMillis;
      ledState = !ledState;
      digitalWrite(LED_BUILTIN, ledState);
      i++;
    }
  }
  digitalWrite(LED_BUILTIN, LOW);
}
