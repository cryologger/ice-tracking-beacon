// ----------------------------------------------------------------------------
// Adafruit LSM303AGR Accelerometer/Magnetomter
// https://www.adafruit.com/product/4413
// ----------------------------------------------------------------------------
void configureLsm303()
{
  // Enable power to IMU
  enableImuPower();

  DEBUG_PRINT("Info: Initializing LSM303AGR + LIS2MDL...");

  // Initialize LSM303AGR accelerometer
  if (lsm303.begin())
  {
    online.lsm303 = true;
    DEBUG_PRINTLN("success!");
  }
  else
  {
    DEBUG_PRINTLN(F("Warning: Failed to initialize LSM303AGR!"));
    online.lsm303 = false;
  }

  // Initialize LIS2MDL magnetometer
  if (lis2mdl.begin())
  {
    online.lis2mdl = true;
  }
  else
  {
    DEBUG_PRINTLN(F("Warning: Failed to initialize LIS2MDL!"));
    online.lis2mdl = false;
  }
}

// Returns a set of scaled magnetic readings from the LIS2MDL
void readLis2mdl(float Mxyz[3])
{
  float temp[3];

  // Read magnetometer
  sensors_event_t event;
  lis2mdl.getEvent(&event);
  Mxyz[0] += event.magnetic.x;
  Mxyz[1] += event.magnetic.y;
  Mxyz[2] += event.magnetic.z;

  // Important: Subtract average of min and max from magnetometer calibration
  Mxyz[0] -= (m_min[0] + m_max[0]) / 2;
  Mxyz[1] -= (m_min[1] + m_max[1]) / 2;
  Mxyz[2] -= (m_min[2] + m_max[2]) / 2;
}

// Returns a heading (°) given an acceleration vector a due to gravity, a magnetic vector m, and a facing vector p (global)
int getHeading(float acc[3], float mag[3], float p[3])
{
  float E[3], N[3]; // Derived direction vectors

  // Compute east vector
  // D x M = E, cross acceleration vector Down with M (magnetic north + inclination)
  vectorCross(mag, acc, E); // Accel vector is up when horizontal
  vectorNormalize(E);

  // Compute north vector
  // E x D = N, cross "East" with "Down" to produce "North" (parallel to the ground)
  vectorCross(acc, E, N);  // Up x East
  vectorNormalize(N);

  // Compute heading in horizontal plane. Get Y and X components of heading from E dot p and N dot p
  int heading = round(atan2(vectorDot(E, p), vectorDot(N, p)) * 180 / M_PI);
  heading = -heading; // Conventional navigation, heading increases North to East
  heading = (heading + 720) % 360; // Apply compass wrap
  return heading;
}

void readLsm303()
{
  // Start loop timer
  unsigned long loopStartTime = millis();

  // Initialize IMU
  configureLsm303();

  // Centered and scaled accelerometer/magnetomer data initialized to zero
  float Axyz[3] = {};
  float Mxyz[3] = {};
  int heading = 0;

  // Check if IMU initialized successfully
  if (online.lsm303 && online.lis2mdl)
  {
    DEBUG_PRINT("Info: Reading LSM303AGR + LIS2MDL...");

    // Read scaled magnetometer data
    readLis2mdl(Mxyz);

    // Read accelerometer data
    sensors_event_t accel;

    lsm303.getEvent(&accel);
    Axyz[0] += accel.acceleration.x;
    Axyz[1] += accel.acceleration.y;
    Axyz[2] += accel.acceleration.z;

    // Calculate pitch, roll and tilt-compensated heading
    float pitch = atan2(-Axyz[0], sqrt(Axyz[1] * Axyz[1] + Axyz[2] * Axyz[2])) * 180 / PI;
    float roll = atan2(Axyz[1], Axyz[2]) * 180 / PI;

    for (byte i = 0; i < 3; i++)
    {
      heading = getHeading(Axyz, Mxyz, p);
      myDelay(100);
    }

    // Write orientation data to union
    moSbdMessage.pitch = pitch * 100;
    moSbdMessage.roll = roll * 100;
    moSbdMessage.heading = heading;

    DEBUG_PRINTLN("done.");

  }
  else
  {
    DEBUG_PRINTLN("Warning: LSM303AGR + LIS2MDL offline!");
  }

  // Disable power to IMU
  disableImuPower();

  // Stop loop timer
  timer.readLsm303 = millis() - loopStartTime;
}

// Basic vector operations
void vectorCross(float a[3], float b[3], float out[3])
{
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}

float vectorDot(float a[3], float b[3])
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vectorNormalize(float a[3])
{
  float mag = sqrt(vectorDot(a, a));
  a[0] /= mag;
  a[1] /= mag;
  a[2] /= mag;
}

// ----------------------------------------------------------------------------
// Adafruit BME280 Temperature Humidity Pressure Sensor
// https://www.adafruit.com/product/2652
// ----------------------------------------------------------------------------
void configureBme280()
{
  DEBUG_PRINT("Info: Initializing BME280...");

  if (bme280.begin())
  {
    online.bme280 = true;
    DEBUG_PRINTLN("success!");
  }
  else
  {
    online.bme280 = false;
    DEBUG_PRINTLN("failed!");
  }
}

// Read BME280
void readBme280()
{
  // Start the loop timer
  unsigned long loopStartTime = millis();

  // Initialize sensor
  configureBme280();

  // Check if sensor initialized successfully
  if (online.bme280)
  {
    DEBUG_PRINT("Info: Reading BME280...");

    myDelay(250);

    // Read sensor data
    temperatureInt = bme280.readTemperature();
    humidityInt = bme280.readHumidity();
    pressureInt = bme280.readPressure() / 100.0F;

    // Write data to union
    moSbdMessage.temperatureInt = temperatureInt * 100;      // Mean internal temperature (°C)
    moSbdMessage.humidityInt    = humidityInt * 100;         // Mean internal humidity (%)
    moSbdMessage.pressureInt    = (pressureInt - 850) * 100; // Mean internal pressure (hPa)

    DEBUG_PRINTLN("done.");
  }
  else
  {
    DEBUG_PRINTLN("Warning: BME280 offline!");
  }
  // Stop the loop timer
  timer.readBme280 = millis() - loopStartTime;
}
