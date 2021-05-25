// Configure attached sensors
void configureSensors()
{
  // Enable power to sensor
  enableSensorPower();

  DEBUG_PRINT("Info: Initializing DPS310...");

  if (dps310.begin_I2C())
  {
    dps310.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
    dps310.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
    online.dps310 = true;
    DEBUG_PRINTLN("success!");
  }
  else
  {
    DEBUG_PRINTLN("failed!");
    online.dps310 = false;
    blinkLed(5, 1000);
  }
}

// Read attached sensors
void readSensors()
{
  // Start the loop timer
  unsigned long loopStartTime = millis();

  // Check if sensor(s) online
  if (online.dps310)
  {
    DEBUG_PRINT("Info: Reading DPS310...");

    sensors_event_t temp_event, pressure_event;

    while (!dps310.temperatureAvailable() || !dps310.pressureAvailable()) {
      return; // wait until there's something to read
    }

    dps310.getEvents(&temp_event, &pressure_event);

    // Record measurements
    float temperature = temp_event.temperature;
    float pressure = pressure_event.pressure;

    // Write data to union
    moSbdMessage.temperature = temperature * 100;
    moSbdMessage.pressure = pressure / 10;

    DEBUG_PRINTLN("done.");
  }
  else
  {
    DEBUG_PRINTLN("Warning: DPS310 offline!");
  }
  // Stop the loop timer
  timer.sensors = millis() - loopStartTime;

  // Disable power to sensor
  disableSensorPower();
}
