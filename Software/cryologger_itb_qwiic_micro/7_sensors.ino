void configureSensors() {
  // SparkFun BME280 Configuration
  if (bme280.beginI2C()) {
    Serial.println(F("SparkFun BME280 detected."));
    bme280.setMode(MODE_SLEEP); // Enter sleep mode
  }
  else {
    Serial.println(F("SparkFun BME280 not connected! Please check wiring."));
  }
}

// Read SparkFun BME280
void readSensors() {

  // Start loop timer
  unsigned long loopStartTime = micros();

  bme280.setMode(MODE_FORCED); // Wake-up, take reading and re-enter sleep mode

  float temperature = bme280.readTempC();
  float humidity = bme280.readFloatHumidity();
  float pressure = bme280.readFloatPressure();

  // Write data to union
  //message.temperature = temperature * 100;
  //message.humidity = humidity * 100;
  //message.pressure = pressure / 10;

  // Stop loop timer
  unsigned long loopEndTime = micros() - loopStartTime;
  Serial.print(F("readBme280() function execution: ")); Serial.print(loopEndTime); Serial.println(F(" μs"));
}
