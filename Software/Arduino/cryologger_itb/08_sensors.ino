/*
  Sensors Module

  This module handles configuration and data acquisition from onboard sensors,
  including temperature, humidity, and pressure from the Adafruit BME280.

  Notes:
  - Temperature and humidity are scaled by 100 before encoding for Iridium SBD
  transmission.
  - Pressure is offset by 850 hPa and scaled by 100 before encoding.
*/

// ----------------------------------------------------------------------------
// No-data sentinels
// ----------------------------------------------------------------------------
static const int16_t BME280_TEMP_NOVALUE = INT16_MIN;
static const uint16_t BME280_HUMIDITY_NOVALUE = UINT16_MAX;
static const uint16_t BME280_PRESSURE_NOVALUE = UINT16_MAX;

// ----------------------------------------------------------------------------
// Configures and reads Adafruit BME280 Temperature Humidity Pressure Sensor.
// More info: https://www.adafruit.com/product/2652
// Attempts initialization up to MAX_TRIES times, then reads temperature,
// humidity, and pressure data.
// ----------------------------------------------------------------------------
void readBme280() {
  // Start execution timer
  uint32_t startTime = millis();

  DEBUG_PRINT("[Sensors] Info: Initializing BME280...");

  // Assume offline unless initialization succeeds
  online.bme280 = false;

  // Attempt to initialize the BME280 sensor
  const int MAX_TRIES = 3;

  for (int attempt = 1; attempt <= MAX_TRIES; ++attempt) {
    if (bme280.begin()) {
      online.bme280 = true;
      break;
    }

    DEBUG_PRINT("[Sensors] Info: BME280 init attempt ");
    DEBUG_PRINT(attempt);
    DEBUG_PRINTLN(" failed. Retrying...");

    myDelay(10);
  }

  if (online.bme280) {
    DEBUG_PRINTLN("success!");
    DEBUG_PRINTLN("[Sensors] Info: Reading BME280.");
    myDelay(250);  // Short delay for stable readings

    // Read raw sensor data
    temperatureInt = bme280.readTemperature();
    humidityInt = bme280.readHumidity();
    pressureInt = bme280.readPressure() / 100.0f;

    // Constrain to valid ranges
    temperatureInt = constrain(temperatureInt, -40.0f, 85.0f);
    humidityInt = constrain(humidityInt, 0.0f, 100.0f);

    // Offset and constrain pressure to valid uint16_t range
    float pressureOffset = pressureInt - 850.0f;
    pressureOffset = constrain(pressureOffset, 0.0f, 655.35f);

    // Write data to SBD structure
    moSbdMessage.temperatureInt = (int16_t)lroundf(temperatureInt * 100.0f);  // °C * 100
    moSbdMessage.humidityInt = (uint16_t)lroundf(humidityInt * 100.0f);       // % * 100
    moSbdMessage.pressureInt = (uint16_t)lroundf(pressureOffset * 100.0f);    // Offset by 850 hPa * 100

  } else {
    online.bme280 = false;

    // Clear globals so printSensors() does not display stale values
    temperatureInt = 0.0f;
    humidityInt = 0.0f;
    pressureInt = 0.0f;

    DEBUG_PRINTLN("failed!");
    DEBUG_PRINTLN("[Sensors] Error: BME280 offline!");

    // Write no-data sentinels to SBD structure
    moSbdMessage.temperatureInt = BME280_TEMP_NOVALUE;
    moSbdMessage.humidityInt = BME280_HUMIDITY_NOVALUE;
    moSbdMessage.pressureInt = BME280_PRESSURE_NOVALUE;
  }

  // Record elapsed execution time
  timer.readBme280 = millis() - startTime;
}