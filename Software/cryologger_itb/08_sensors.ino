// ----------------------------------------------------------------------------
// Adafruit LSM303AGR Accelerometer/Magnetomter
// https://www.adafruit.com/product/4413
// ----------------------------------------------------------------------------
void configureLsm303()
{
  DEBUG_PRINT("Info: Initializing LSM303...");

  // Initialize LSM303 accelerometer
  if (lsm303.begin())
  {
    online.lsm303 = true;
    DEBUG_PRINTLN("success!");
  }
  else
  {
    online.lsm303 = false;
    DEBUG_PRINTLN("failed!");
  }
}

void readLsm303()
{
  // Start loop timer
  unsigned long loopStartTime = millis();

  // Initialize accelerometer
  configureLsm303();

  // Check if sensor initialized successfully
  if (online.lsm303)
  {
    DEBUG_PRINT("Info: Reading LSM303...");

    myDelay(500);

    // Read accelerometer data
    sensors_event_t accel;
    lsm303.getEvent(&accel);

    // Calculate pitch and roll
    float pitch = atan2(-accel.acceleration.x, sqrt(accel.acceleration.y * accel.acceleration.y + accel.acceleration.z * accel.acceleration.z)) * 180 / PI;
    float roll = atan2(accel.acceleration.y, accel.acceleration.z) * 180 / PI;
    
    // Write data to union
    moSbdMessage.pitch = pitch * 100;
    moSbdMessage.roll = roll * 100;
    //moSbdMessage.heading = heading * 10;

    // Add to statistics object
    //pitchStats.add();
    //rollStats.add();
    //headingStats.add();
    
    DEBUG_PRINTLN("done.");

    // Print debug info
    //DEBUG_PRINT(F("pitch: ")); DEBUG_PRINT_DEC(pitch, 2);
    //DEBUG_PRINT(F(" roll: ")); DEBUG_PRINTLN_DEC(roll, 2);

  }
  else
  {
    DEBUG_PRINTLN("Warning: LSM303 offline!");
  }

  // Stop loop timer
  timer.readLsm303 = millis() - loopStartTime;
}

// ----------------------------------------------------------------------------
// Adafruit LSM6DS33 + LIS3MDL - 9 DoF IMU
// https://www.adafruit.com/product/4485
// ----------------------------------------------------------------------------
// Configure
void configureLsm6ds33()
{
  // Enable power to IMU
  enableImuPower();

  DEBUG_PRINT("Info: Initializing LSM6DS33 + LIS3MDL...");

  // Initialize LSM6DS33 accelerometer/gyroscope
  if (lsm6ds33.begin_I2C())
  {
    online.lsm6ds33 = true;
    DEBUG_PRINTLN("success!");
  }
  else
  {
    DEBUG_PRINTLN(F("Warning: Failed to initialize LSM6DS33!"));
    online.lsm6ds33 = false;
  }

  // Initialize LIS3MDL magnetometer
  if (lis3mdl.begin_I2C())
  {
    online.lis3mdl = true;
  }
  else
  {
    DEBUG_PRINTLN(F("Warning: Failed to initialize LIS3MDL!"));
    online.lis3mdl = false;
  }
}

// Returns a set of scaled magnetic readings from the LIS3MDL
void readLis3mdl(float Mxyz[3])
{
  float temp[3];

  // Read magnetometer
  lis3mdl.read();
  Mxyz[0] += lis3mdl.x;
  Mxyz[1] += lis3mdl.y;
  Mxyz[2] += lis3mdl.z;

  // Apply offsets (bias) and scale factors from Magneto
  for (byte i = 0; i < 3; i++)
  {
    temp[i] = (Mxyz[i] - M_B[i]);
  }
  Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
  Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
  Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
  vectorNormalize(Mxyz);
}

// Returns a heading (in degrees) given an acceleration vector a due to gravity, a magnetic vector m, and a facing vector p (global)
int getHeading(float acc[3], float mag[3], float p[3])
{
  float E[3], N[3]; // Derived direction vectors

  // D x M = E, cross acceleration vector Down with M (magnetic north + inclination) to produce "East"
  vectorCross(mag, acc, E); // Accel vector is up when horizontal
  vectorNormalize(E);

  // E x D = N, cross "East" with "Down" to produce "North" (parallel to the ground)
  vectorCross(acc, E, N);  // Up x East
  vectorNormalize(N);

  // Compute heading in horizontal plane. Get Y and X components of heading from E dot p and N dot p
  int heading = round(atan2(vectorDot(E, p), vectorDot(N, p)) * 180 / M_PI);
  heading = -heading; // Conventional navigation, heading increases North to East
  heading = (heading + 720) % 360; // Apply compass wrap
  return heading;
}

void readLsm6ds33()
{
  // Start loop timer
  unsigned long loopStartTime = millis();

  // Initialize IMU
  configureLsm6ds33();

  // Centered and scaled accelerometer/magnetomer data initialized to zero
  float Axyz[3] = {};
  float Mxyz[3] = {};
  int heading = 0;

  // Check if IMU initialized successfully
  if (online.lsm6ds33 && online.lis3mdl)
  {
    DEBUG_PRINT("Info: Reading LSM6DS33 + LIS3MDL...");

    // Read scaled magnetometer data
    readLis3mdl(Mxyz);

    // Read normalized accelerometer data
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;

    lsm6ds33.getEvent(&accel, &gyro, &temp);
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
    DEBUG_PRINTLN("Warning: LSM6DS33 + LIS3MDL offline!");
  }

  // Disable power to IMU
  disableImuPower();

  // Stop loop timer
  timer.imu = millis() - loopStartTime;
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

// ----------------------------------------------------------------------------
// Adafruit DPS310 Precision Barometric Pressure Sensor
// https://www.adafruit.com/product/4494
// ----------------------------------------------------------------------------
void configureDps310()
{
  DEBUG_PRINT("Info: Initializing DPS310...");

  if (dps310.begin_I2C())
  {
    online.dps310 = true;
    DEBUG_PRINTLN("success!");
    dps310.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
    dps310.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
  }
  else
  {
    online.dps310 = false;
    DEBUG_PRINTLN("failed!");
  }
}

// Read DPS310
void readDps310()
{
  // Start the loop timer
  unsigned long loopStartTime = millis();

  // Initialize sensor(s)
  configureDps310();

  // Check if sensor initialized successfully
  if (online.dps310)
  {
    DEBUG_PRINT("Info: Reading DPS310...");

    sensors_event_t temp_event, pressure_event;

    // Read sensor until value is returned or timeout is exceeded
    while ((!dps310.temperatureAvailable() || !dps310.pressureAvailable()) && millis() - loopStartTime < 5000UL)
    {
      return;
    }

    // Read sensor data
    dps310.getEvents(&temp_event, &pressure_event);
    float temperatureInt = temp_event.temperature;

    // Write data to union
    moSbdMessage.temperatureInt = temperatureInt * 100;

    DEBUG_PRINTLN("done.");
  }
  else
  {
    DEBUG_PRINTLN("Warning: DPS310 offline!");
  }
  // Stop the loop timer
  timer.readDps310 = millis() - loopStartTime;
}
