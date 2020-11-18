// Configure attached sensors
void configureSensors() {

  // SparkFun BME280 Configuration
  if (bme280.beginI2C()) {
    bme280.setMode(MODE_SLEEP); // Enter sleep mode
  }
  else {
    Serial.println(F("Warning: SparkFun BME280 not connected! Please check wiring."));
  }
}

// Read attached sensors
void readSensors() {
  setPixelColour(yellow);

  unsigned long loopStartTime = millis();

  // Wake-up, take readings and re-enter sleep mode
  bme280.setMode(MODE_FORCED);
  float temperature = bme280.readTempC();
  float humidity = bme280.readFloatHumidity() / 1000;
  float pressure = bme280.readFloatPressure();

  // Write data to union
  message.temperature = temperature * 100;
  message.humidity = humidity * 100;
  message.pressure = pressure;

  unsigned long loopEndTime = millis() - loopStartTime;
  Serial.print(F("readSenors() function execution: ")); Serial.print(loopEndTime); Serial.println(F(" ms"));
}
