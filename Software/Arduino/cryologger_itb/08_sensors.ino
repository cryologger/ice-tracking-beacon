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
// Configures and reads Adafruit BME280 Temperature Humidity Pressure Sensor.
// More info: https://www.adafruit.com/product/2652
// Initializes the BME280 sensor and reads temperature, humidity, and
// pressure data.
// ----------------------------------------------------------------------------
void readBme280() {
  // Start execution timer
  unsigned long startTime = millis();

  DEBUG_PRINT("[Sensors] Info: Initializing BME280...");

  // Attempt to initialize the BME280 sensor
  if (bme280.begin()) {
    online.bme280 = true;
    DEBUG_PRINTLN("success!");

    DEBUG_PRINTLN("[Sensors] Info: Reading BME280.");
    myDelay(250);  // Short delay for stable readings

    // Read raw sensor data
    temperatureInt = bme280.readTemperature();
    humidityInt = bme280.readHumidity();
    pressureInt = bme280.readPressure() / 100.0F;

    // Write data to SBD structure
    moSbdMessage.temperatureInt = temperatureInt * 100;    // °C * 100
    moSbdMessage.humidityInt = humidityInt * 100;          // % * 100
    moSbdMessage.pressureInt = (pressureInt - 850) * 100;  // Offset by 850 hPa * 100

  } else {
    online.bme280 = false;
    DEBUG_PRINTLN("failed!");
    DEBUG_PRINTLN("[Sensors] Warning: BME280 offline!");
  }

  // Record elapsed execution time.
  timer.readBme280 = millis() - startTime;
}
