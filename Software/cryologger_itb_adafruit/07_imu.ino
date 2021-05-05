// Configure IMU
void configureImu()
{
  DEBUG_PRINT("Info: Initializing IMU...");

  // Enable power to IMU GPIO pin
  enableImuPower();

  // Initialize LSM6DS33 accelerometer/gyroscope
  if (imu.init())
  {
    online.imu = true;
    imu.enableDefault();
    DEBUG_PRINTLN("success!");
  }
  else
  {
    DEBUG_PRINTLN(F("Warning: Failed to initialize LSM6DS33!"));
    online.imu = false;
  }

  // Initialize LIS3MDL magnetometer
  if (mag.init())
  {
    mag.enableDefault();
    online.mag = true;
  }
  else
  {
    DEBUG_PRINTLN(F("Warning: Failed to initialize LIS3MDL!"));
    online.mag = false;
  }

}

// Returns a set of scaled magnetic readings from the LIS3MDL
void read_data(vector * m)
{
  static float x, y, z;
  mag.read();
  x = mag.m.x - B[0];
  y = mag.m.y - B[1];
  z = mag.m.z - B[2];
  m->x = Ainv[0][0] * x + Ainv[0][1] * y + Ainv[0][2] * z;
  m->y = Ainv[1][0] * x + Ainv[1][1] * y + Ainv[1][2] * z;
  m->z = Ainv[2][0] * x + Ainv[2][1] * y + Ainv[2][2] * z;
}

// Returns a heading (in degrees) given an acceleration vector a due to gravity, a magnetic vector m, and a facing vector p (global).
int get_heading(const vector * a, const vector * m)
{
  vector E;
  vector N;

  // D X M = E, cross acceleration vector Down with M (magnetic north + inclination) to produce "East"
  vector_cross(m, a, &E); // Accel vector is up when horizontal
  vector_normalize(&E);

  // E X D = N, cross "East" with "Down" to produce "North" (parallel to the ground)
  vector_cross(a, &E, &N);  // Up x East
  vector_normalize(&N);

  // Compute heading, get Y and X components of heading from E dot p and N dot p
  int heading = round(atan2(vector_dot(&E, &p), vector_dot(&N, &p)) * 180 / M_PI);
  if (heading < 0)
    heading += 360;
  return heading;
}

void readImu()
{
  // Start loop timer
  unsigned long loopStartTime = millis();

  // Check if IMU initialized successfully
  if (online.imu && online.mag)
  {
    // Change LED colour
    //setLedColour(CRGB::Blue);

    DEBUG_PRINT("Info: Reading IMU...");

    vector a, m;
    int heading;
    float scl = 1.0 / 16384.0; // 1 g (0.061 / LSB) / 1000
    read_data(&m);

    imu.read();

    // Normalize accelerometer data
    a.x = imu.a.x * scl;
    a.y = imu.a.y * scl;
    a.z = imu.a.z * scl;

    // Calculate pitch, roll and heading
    float pitch = atan2(-a.x, sqrt(a.y * a.y + a.z * a.z)) * 180 / PI;
    float roll = atan2(a.y, a.z) * 180 / PI;
    heading = get_heading(&a, &m);

    // Write orientation data to union
    moSbdMessage.pitch = pitch * 100;
    moSbdMessage.roll = roll * 100;
    moSbdMessage.heading = heading;

    DEBUG_PRINTLN("done.");

  }
  else
  {
    DEBUG_PRINTLN("Warning: LSM6DS33 and/or LIS3MDL offline!");
  }
  
  // Disable pin power to IMU
  disableImuPower();

  // Stop loop timer
  timer.imu = millis() - loopStartTime;
}
