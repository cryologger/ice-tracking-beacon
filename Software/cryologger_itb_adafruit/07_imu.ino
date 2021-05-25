// Configure IMU
void configureImu()
{
  // Enable power to IMU
  enableImuPower();

  DEBUG_PRINT("Info: Initializing IMU...");

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
  for (byte i = 0; i < samples; i++)   // 30 samples
  {
    lis3mdl.read();
    Mxyz[0] += lis3mdl.x;
    Mxyz[1] += lis3mdl.y;
    Mxyz[2] += lis3mdl.z;
    delay(1);
  }

  // Average magnetometer readings
  Mxyz[0] /= samples;
  Mxyz[0] /= samples;
  Mxyz[0] /= samples;

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

void readImu()
{
  // Start loop timer
  unsigned long loopStartTime = millis();

  // Centered and scaled accelerometer/magnetomer data initialized to zero
  float Axyz[3] = {};
  float Mxyz[3] = {};

  // Check if IMU initialized successfully
  if (online.lsm6ds33 && online.lis3mdl)
  {
    DEBUG_PRINT("Info: Reading IMU...");

    // Read scaled magnetometer data
    readLis3mdl(Mxyz);

    // Read normalized accelerometer data
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    
    for (byte i = 0; i < samples; i++)
    {
      lsm6ds33.getEvent(&accel, &gyro, &temp);
      Axyz[0] += accel.acceleration.x;
      Axyz[1] += accel.acceleration.y;
      Axyz[2] += accel.acceleration.z;
      delay(1);
    }

    // Average accelerometer readings
    Axyz[0] /= samples;
    Axyz[1] /= samples;
    Axyz[2] /= samples;

    // Calculate pitch, roll and tilt-compensated heading
    float pitch = atan2(-Axyz[0], sqrt(Axyz[1] * Axyz[1] + Axyz[2] * Axyz[2])) * 180 / PI;
    float roll = atan2(Axyz[1], Axyz[2]) * 180 / PI;
    int heading = getHeading(Axyz, Mxyz, p);

    // Write orientation data to union
    moSbdMessage.pitch = pitch * 100;
    moSbdMessage.roll = roll * 100;
    moSbdMessage.heading = heading;

    DEBUG_PRINTLN("done.");

  }
  else
  {
    DEBUG_PRINTLN("Warning: IMU offline!");
  }

  // Disable power to IMU
  //disableImuPower();

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
