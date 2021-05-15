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
void read_data(float Mxyz[3])
{
  byte i;
  float temp[3];
  lis3mdl.read();

  Mxyz[0] = lis3mdl.x;
  Mxyz[1] = lis3mdl.y;
  Mxyz[2] = lis3mdl.z;

  // Apply offsets (bias) and scale factors from Magneto
  for (int i = 0; i < 3; i++) temp[i] = (Mxyz[i] - M_B[i]);
  Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
  Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
  Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
  vector_normalize(Mxyz);
}

// Returns a heading (in degrees) given an acceleration vector a due to gravity, a magnetic vector m, and a facing vector p (global).
int get_heading(float acc[3], float mag[3], float p[3])
{
  float E[3], N[3]; // Derived direction vectors

  // D X M = E, cross acceleration vector Down with M (magnetic north + inclination) to produce "East"
  vector_cross(mag, acc, E); // Accel vector is up when horizontal
  vector_normalize(E);

  // E X D = N, cross "East" with "Down" to produce "North" (parallel to the ground)
  vector_cross(acc, E, N);  // Up x East
  vector_normalize(N);

  // Compute heading in horizontal plane. Get Y and X components of heading from E dot p and N dot p
  int heading = round(atan2(vector_dot(E, p), vector_dot(N, p)) * 180 / M_PI);
  heading = -heading; // Conventional navigation, heading increases North to East
  heading = (heading + 720) % 360; // Apply compass wrap
  return heading;
}

void readImu()
{
  // Start loop timer
  unsigned long loopStartTime = millis();

  float Axyz[3], Mxyz[3]; // Centered and scaled accelerometer/magnetomer data

  // Check if IMU initialized successfully
  if (online.lsm6ds33 && online.lis3mdl)
  {
    DEBUG_PRINT("Info: Reading IMU...");

    // Read scaled magnetometer data
    read_data(Mxyz);

    // Read accelerometer
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    lsm6ds33.getEvent(&accel, &gyro, &temp);

    // Normalized accelerometer data
    Axyz[0] = accel.acceleration.x;
    Axyz[1] = accel.acceleration.y;
    Axyz[2] = accel.acceleration.z;

    // Calculate pitch, roll and heading
    float pitch = atan2(-Axyz[0], sqrt(Axyz[1] * Axyz[1] + Axyz[2] * Axyz[2])) * 180 / PI;
    float roll = atan2(Axyz[1], Axyz[2]) * 180 / PI;
    int heading = get_heading(Axyz, Mxyz, p);

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
  disableImuPower();

  // Stop loop timer
  timer.imu = millis() - loopStartTime;
}

// Basic vector operations
void vector_cross(float a[3], float b[3], float out[3])
{
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}

float vector_dot(float a[3], float b[3])
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vector_normalize(float a[3])
{
  float mag = sqrt(vector_dot(a, a));
  a[0] /= mag;
  a[1] /= mag;
  a[2] /= mag;
}
