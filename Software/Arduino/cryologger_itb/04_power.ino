/*
  Power Module

  This module provides control over power for hardware components,
  including battery voltage measurement, toggling serial communication,
  and enabling/disabling power to GNSS, IMU, sensors, and RockBLOCK 9603.
  It also includes a non-blocking LED blink function, a non-blocking delay
  for watchdog resets, and routines to prepare or recover from deep sleep.
*/

// ----------------------------------------------------------------------------
// Measures battery voltage from the 10/1 MΩ divider.
// ----------------------------------------------------------------------------
float readBattery() {
  // Start execution timer
  unsigned long startTime = millis();

  // Measure external battery voltage across 10/1 MΩ resistor divider (1/10)
  (void)analogRead(PIN_VBAT);  // Dummy read
  voltage = analogRead(PIN_VBAT);
  voltage *= ((10000000.0 + 1000000.0) / 1000000.0);
  voltage *= 3.3;   // Multiply by 3.3V reference
  voltage /= 4096;  // Convert raw ADC count to actual voltage

  // Record elapsed execution time
  timer.readBattery = millis() - startTime;
  return voltage;
}

// ----------------------------------------------------------------------------
// Disables the serial port.
// ----------------------------------------------------------------------------
void disableSerial() {
#if DEBUG
  SERIAL_PORT.end();   // Close the Serial port
  USBDevice.detach();  // Detach the USB interface prior to sleeping
#endif
}

// ----------------------------------------------------------------------------
// Enables the serial port.
// ----------------------------------------------------------------------------
void enableSerial() {
#if DEBUG
  USBDevice.attach();         // Attach the USB interface
  SERIAL_PORT.begin(115200);  // Open the Serial port
  // myDelay(3000);
#endif
}

// ----------------------------------------------------------------------------
// Enables power to the IMU.
// Includes a short non-blocking delay for sensor stabilization.
// ----------------------------------------------------------------------------
void enableImuPower() {
  digitalWrite(PIN_IMU_EN, HIGH);
  myDelay(500);
}

// ----------------------------------------------------------------------------
// Disables power to the IMU.
// ----------------------------------------------------------------------------
void disableImuPower() {
  digitalWrite(PIN_IMU_EN, LOW);
}

// ----------------------------------------------------------------------------
// Enable power to the sensors.
// Includes a short non-blocking delay for sensor stabilization.
// ----------------------------------------------------------------------------
void enableSensorPower() {
  digitalWrite(PIN_SENSOR_EN, HIGH);
  myDelay(500);
}

// ----------------------------------------------------------------------------
// Disables power to the sensors.
// ----------------------------------------------------------------------------
void disableSensorPower() {
  digitalWrite(PIN_SENSOR_EN, LOW);
}

// ----------------------------------------------------------------------------
// Enables power to GNSS.
// Includes a longer non-blocking delay to ensure GNSS module is fully powered.
// ----------------------------------------------------------------------------
void enableGnssPower() {
  digitalWrite(PIN_GNSS_EN, LOW);
  myDelay(1000);
}

// ----------------------------------------------------------------------------
// Disable power to GNSS.
// ----------------------------------------------------------------------------
void disableGnssPower() {
  digitalWrite(PIN_GNSS_EN, HIGH);
}

// ----------------------------------------------------------------------------
// Enables 5V power to the RockBLOCK 9603.
// ----------------------------------------------------------------------------
void enable5V() {
  digitalWrite(PIN_5V_EN, HIGH);
  myDelay(500);
}

// ----------------------------------------------------------------------------
// Disables 5V power to the RockBLOCK 9603.
// ----------------------------------------------------------------------------
void disable5V() {
  digitalWrite(PIN_5V_EN, LOW);
}

// ----------------------------------------------------------------------------
// Prepares the system for deep sleep.
// ----------------------------------------------------------------------------
void prepareForSleep() {
  disableSerial();

  // Clear structures
  online = {};
  timer = {};
}

// ----------------------------------------------------------------------------
// Enters deep sleep until an RTC or WDT interrupt occurs.
// ----------------------------------------------------------------------------
void goToSleep() {
  if (firstTimeFlag) {
    firstTimeFlag = false;
  }
  LowPower.deepSleep();
  /*
     Execution halts here until an RTC/WDT interrupt wakes the device
  */
}

// ----------------------------------------------------------------------------
// Wake up from deep sleep.
// ----------------------------------------------------------------------------
void wakeUp() {
  enableSerial();  // Re-enable the serial port
}

// ----------------------------------------------------------------------------
// Non-blocking LED blink routine.
// Flashes the built-in LED a specified number of times with the given delay.
// ----------------------------------------------------------------------------
void blinkLed(byte ledFlashes, unsigned int ledDelay) {
  byte i = 0;
  while (i < ledFlashes * 2) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= ledDelay) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      previousMillis = currentMillis;
      i++;
    }
  }
  digitalWrite(LED_BUILTIN, LOW);  // Ensure LED is off after blinking
}

// ----------------------------------------------------------------------------
// Non-blocking delay function that continues to reset the Watchdog Timer.
// This function delays for a specified duration (in milliseconds) while
// calling petDog() to prevent unintended WDT resets.
// ----------------------------------------------------------------------------
void myDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    resetWdt();  // Reset the WDT during the delay
  }
}
