/*
  IMU Module

  This module handles configuration and data acquisition from the LSM6DSOX
  accelerometer/gyroscope and LI32MDL magnetometer. It computes pitch, roll,
  and tilt-compensated heading, and stores the data in the MO-SBD message.
*/

// ----------------------------------------------------------------------------
// Configures the Adafruit LSM6DSOX + LIS3MDL - Precision 9 DoF IMU
// More info: https://www.adafruit.com/product/4517
// ----------------------------------------------------------------------------
void configureLsm6dsox() {

  enableImuPower();  // Enable power to IMU

  DEBUG_PRINT("[IMU] Info: Initializing LSM6DSOX + LIS3MDL...");

  // Initialize LSM6DSOX accelerometer
  if (lsm6dsox.begin_I2C()) {
    online.lsm6dsox = true;
    DEBUG_PRINTLN("success!");
  } else {
    online.lsm6dsox = false;
    DEBUG_PRINTLN(F("[IMU] Warning: Failed to initialize LSM6DSOX!"));
  }

  // Initialize LIS2MDL magnetometer
  if (lis3mdl.begin_I2C()) {
    online.lis3mdl = true;
  } else {
    online.lis3mdl = false;
    DEBUG_PRINTLN(F("[IMU] Warning: Failed to initialize LIS3MDL!"));
  }
}

// ----------------------------------------------------------------------------
// Reads the Adafruit LSM6DSOX + LIS3MDL - Precision 9 DoF IMU
// ----------------------------------------------------------------------------
void readLsm6dsox() {
  // Start execution timer
  unsigned long loopStartTime = millis();

  // Initialize IMU
  configureLsm6dsox();

  // Centered and scaled accelerometer/magnetomer data initialized to zero
  float Axyz[3] = {};
  float Mxyz[3] = {};
  int heading = 0;

  // Check if IMU initialized successfully
  if (online.lsm6dsox && online.lis3mdl) {
    DEBUG_PRINTLN("[IMU] Reading LSM6DSOX + LIS3MDL.");

    // Read scaled magnetometer data
    readLis3mdl(Mxyz);

    // Read accelerometer data
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    lsm6dsox.getEvent(&accel, &gyro, &temp);

    Axyz[0] += accel.acceleration.x;
    Axyz[1] += accel.acceleration.y;
    Axyz[2] += accel.acceleration.z;

    // Calculate pitch, roll and tilt-compensated heading
    pitch = atan2(-Axyz[0], sqrt(Axyz[1] * Axyz[1] + Axyz[2] * Axyz[2])) * 180 / PI;
    roll = atan2(Axyz[1], Axyz[2]) * 180 / PI;

    for (byte i = 0; i < 3; i++) {
      heading = getHeading(Axyz, Mxyz, p);
      myDelay(100);
    }

    // Write orientation data to union
    moSbdMessage.pitch = pitch * 100;
    moSbdMessage.roll = roll * 100;
    moSbdMessage.heading = heading;

  } else {
    DEBUG_PRINTLN("[IMU] Warning: LSM6DSOX + LIS3MDL offline!");
  }

  // Disable power to IMU
  disableImuPower();

  // Record elapsed execution time
  timer.readLsm6dsox = millis() - loopStartTime;
}

// Returns a set of scaled magnetic readings from the LIS3MDL
void readLis3mdl(float Mxyz[3]) {
  //float temp[3];

  // Read magnetometer
  sensors_event_t event;
  lis3mdl.getEvent(&event);
  Mxyz[0] += event.magnetic.x;
  Mxyz[1] += event.magnetic.y;
  Mxyz[2] += event.magnetic.z;

  // Important: Subtract average of min and max from magnetometer calibration
  Mxyz[0] -= (m_min[0] + m_max[0]) / 2;
  Mxyz[1] -= (m_min[1] + m_max[1]) / 2;
  Mxyz[2] -= (m_min[2] + m_max[2]) / 2;
}

// ----------------------------------------------------------------------------
// Returns a heading [0..359] (degrees given an acceleration vector a due to 
// gravity, a magnetic vector m, and a facing vector p.
// ----------------------------------------------------------------------------
int getHeading(float acc[3], float mag[3], float p[3]) {
  float W[3], N[3];  // Derived direction vectors

  // "Up" is 'acc', "mag" is Earth's magnetic field
  // cross(Up, mag) => "West"
  vectorCross(acc, mag, W);
  vectorNormalize(W);

  // Cross(West, Up) => "North" (parallel to ground)
  vectorCross(W, acc, N);
  vectorNormalize(N);

  // Compute heading in horizontal plane. Correct for local magnetic declination.
  int heading = round(atan2(vectorDot(W, p), vectorDot(N, p)) * 180.0 / M_PI ); // + declination
  heading = -heading;               // Conventional nav: heading increases clockwise
  heading = (heading + 720) % 360;  // Apply compass wrap to [0..359]
  return heading;
}

// ----------------------------------------------------------------------------
// Vector math helpers
// ----------------------------------------------------------------------------
void vectorCross(float a[3], float b[3], float out[3]) {
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}

float vectorDot(float a[3], float b[3]) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vectorNormalize(float a[3]) {
  float m = sqrt(vectorDot(a, a));
  if (m != 0) {
    a[0] /= m;
    a[1] /= m;
    a[2] /= m;
  }
}
