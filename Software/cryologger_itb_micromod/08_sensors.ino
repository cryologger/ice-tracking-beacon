// Configure attached sensors
void configureSensors()
{
  // SparkFun BME280
  if (bme280.beginI2C())
  {
    bme280.setMode(MODE_SLEEP); // Enter sleep mode
    DEBUG_PRINTLN("Info: BME280 initialized");
    online.bme280 = true;
  }
  else
  {
    DEBUG_PRINTLN("Warning: SparkFun BME280 not detected at default I2C address! Please check wiring.");
    online.bme280 = false;
  }
}

// Read attached sensors
void readSensors()
{
  // Start the loop timer
  unsigned long loopStartTime = millis();

  if (online.bme280)
  {
    // Wake sensor and return to sleep once the measurement is made
    bme280.setMode(MODE_FORCED);

    // Delay required to allow sensor to perform the measurement
    while (bme280.isMeasuring() && millis() - loopStartTime < 1000);

    // Record measurements
    float temperature = bme280.readTempC();
    float humidity = bme280.readFloatHumidity();
    float pressure = bme280.readFloatPressure();

    // Write data to union
    moMessage.temperature = temperature * 100;
    moMessage.humidity = humidity * 100;
    moMessage.pressure = pressure / 10;
  }
  else
  {
    DEBUG_PRINTLN("Warning: BME280 offline!");
  }

  // Stop the loop timer
  timer.sensor = millis() - loopStartTime;
}
