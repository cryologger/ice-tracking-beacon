/*
  IMU Module

  This module handles configuration and data acquisition from the LSM6DSOX
  accelerometer/gyroscope and LIS3MDL magnetometer. It computes pitch, roll,
  and tilt-compensated heading, and stores the data in the MO-SBD message.
*/

#include "imu_calibration.h"

// ----------------------------------------------------------------------------
// Per-unit calibration & frame config (PASTE YOUR NUMBERS)
// ----------------------------------------------------------------------------

// Forward (body) vector in sensor frame. Change if your board orientation differs.
// {1, 0, 0});    // Align to X+
// {-1, 0, 0});   // Align to X-
// {0, 1, 0});    // Align to Y+
// {0, -1, 0});   // Align to Y-
// {0, 0, 1});    // Align to Z+
// {0, 0, -1});   // Align to Z-
static float p[3] = { 1.0, 0.0, 0.0 };  // Normalized once at boot

// Magnetometer 3×3 calibration (Magneto-style). REPLACE with your unit’s values.
// Bias (µT)
static float M_B[3] = { 0.0, 0.0, 0.0 };

// Inverse soft-iron matrix
static float M_Ainv[3][3] = {
  { 1.0, 0.0, 0.0 },
  { 0.0, 1.0, 0.0 },
  { 0.0, 0.0, 1.0 }
};

// One-time init (normalize p)
static void initOnce() {
  static bool inited = false;
  if (inited) return;
  inited = true;

  // Normalize forward vector p once
  float m2 = p[0] * p[0] + p[1] * p[1] + p[2] * p[2];
  if (m2 > 0.0) {
    float inv = 1.0 / sqrtf(m2);
    p[0] *= inv;
    p[1] *= inv;
    p[2] *= inv;
  } else {
    p[0] = 1.0;
    p[1] = 0.0;
    p[2] = 0.0;
  }
}

// ----------------------------------------------------------------------------
// Loads calibration data.
// ----------------------------------------------------------------------------
static void selectImuCalibrationOnce() {
  static bool done = false;
  if (done) return;
  done = true;

  const MagCal* C = getSelectedCal();
  if (!C) {
    DEBUG_PRINT("[IMU] Warning: No calibration found for UID=");
    DEBUG_PRINTLN(UID);
    return;
  }

  // Copy calibration into working variables
  for (int i = 0; i < 3; ++i) {
    M_B[i] = C->B[i];
    p[i] = C->p[i];
    for (int j = 0; j < 3; ++j) {
      M_Ainv[i][j] = C->Ainv[i][j];
    }
  }

  // Normalize forward vector p
  float m2 = p[0] * p[0] + p[1] * p[1] + p[2] * p[2];
  if (m2 > 0.0f) {
    float inv = 1.0 / sqrtf(m2);
    p[0] *= inv;
    p[1] *= inv;
    p[2] *= inv;
  } else {
    p[0] = 1.0;
    p[1] = 0.0;
    p[2] = 0.0;
  }

  DEBUG_PRINT("[IMU] Using calibration for ");
  DEBUG_PRINTLN(C->uid);
}

// ----------------------------------------------------------------------------
// Configures the LSM6DSOX + LIS3MDL IMU.
// ----------------------------------------------------------------------------
void configureLsm6dsox() {
  enableImuPower();  // Enable power to IMU rail
  myDelay(5);        // Rail settle

  // One-time setup (safe to call every wake)
  initOnce();
  selectImuCalibrationOnce();

  DEBUG_PRINT("[IMU] Info: Initializing LSM6DSOX + LIS3MDL...");

  const int MAX_TRIES = 3;
  online.lsm6dsox = false;
  online.lis3mdl = false;

  for (int attempt = 1; attempt <= MAX_TRIES; ++attempt) {
    // LSM6DSOX
    if (!online.lsm6dsox) {
      if (lsm6dsox.begin_I2C()) {
        online.lsm6dsox = true;
        lsm6dsox.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
        lsm6dsox.setGyroDataRate(LSM6DS_RATE_SHUTDOWN);
        lsm6dsox.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
        myDelay(2);
      }
    }
    // LIS3MDL
    if (!online.lis3mdl) {
      if (lis3mdl.begin_I2C()) {
        online.lis3mdl = true;
        lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
        lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
        lis3mdl.setDataRate(LIS3MDL_DATARATE_10_HZ);
        lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
        myDelay(2);
      }
    }

    if (online.lsm6dsox && online.lis3mdl) break;

    DEBUG_PRINT("[IMU] Info: init attempt ");
    DEBUG_PRINT(attempt);
    DEBUG_PRINTLN(" failed. Retrying...");
    myDelay(10);
  }

  if (online.lsm6dsox) DEBUG_PRINTLN("success!");
  else DEBUG_PRINTLN(F("[IMU] Warning: Failed to initialize LSM6DSOX!"));
  if (!online.lis3mdl) DEBUG_PRINTLN(F("[IMU] Warning: Failed to initialize LIS3MDL!"));
}

// ----------------------------------------------------------------------------
// Reads the LSM6DSOX + LIS3MDL IMU.
// ----------------------------------------------------------------------------
void readLsm6dsox() {
  // Start execution timer
  unsigned long startTime = millis();

  // Reconfigure after each wake
  configureLsm6dsox();

  float Axyz[3] = { 0.0, 0.0, 0.0 };
  float Mxyz[3] = { 0.0, 0.0, 0.0 };  // corrected mag (3×3 bias/scale applied)
  int heading_deg = 0;

  if (online.lsm6dsox && online.lis3mdl) {
    DEBUG_PRINTLN("[IMU] Reading LSM6DSOX + LIS3MDL.");

    // Discard first few samples after power-up to avoid startup transients
    discardImuWarmupSamples(6);  // simple fixed-count discard (≈10 ms each inside)

    // Average a few samples for stability (timing unchanged)
    const int N = 5;
    sensors_event_t accel, gyro, temp;

    for (int i = 0; i < N; ++i) {
      // Magnetometer (3×3 calibrated)
      sensors_event_t magEvt;
      lis3mdl.getEvent(&magEvt);
      float mx_raw[3] = { magEvt.magnetic.x, magEvt.magnetic.y, magEvt.magnetic.z };
      float m_tmp[3] = { mx_raw[0] - M_B[0], mx_raw[1] - M_B[1], mx_raw[2] - M_B[2] };
      float mx = M_Ainv[0][0] * m_tmp[0] + M_Ainv[0][1] * m_tmp[1] + M_Ainv[0][2] * m_tmp[2];
      float my = M_Ainv[1][0] * m_tmp[0] + M_Ainv[1][1] * m_tmp[1] + M_Ainv[1][2] * m_tmp[2];
      float mz = M_Ainv[2][0] * m_tmp[0] + M_Ainv[2][1] * m_tmp[1] + M_Ainv[2][2] * m_tmp[2];

      Mxyz[0] += mx;
      Mxyz[1] += my;
      Mxyz[2] += mz;

      // Accelerometer
      lsm6dsox.getEvent(&accel, &gyro, &temp);
      Axyz[0] += accel.acceleration.x;
      Axyz[1] += accel.acceleration.y;
      Axyz[2] += accel.acceleration.z;

      myDelay(5);
    }

    // Average
    const float invN = 1.0 / (float)N;
    Axyz[0] *= invN;
    Axyz[1] *= invN;
    Axyz[2] *= invN;
    Mxyz[0] *= invN;
    Mxyz[1] *= invN;
    Mxyz[2] *= invN;

    // Normalize Up and Mag. Bail safely if degenerate.
    if (!normalizeSafe(Axyz)) {
      DEBUG_PRINTLN("[IMU] Warning: accel norm too small, orientation unreliable.");
    }
    if (!normalizeSafe(Mxyz)) {
      DEBUG_PRINTLN("[IMU] Warning: mag norm too small, heading unreliable.");
    }

    // Compute pitch and roll from accel
    pitch = atan2f(-Axyz[0], sqrtf(Axyz[1] * Axyz[1] + Axyz[2] * Axyz[2])) * 180.0 / M_PI;
    roll = atan2f(Axyz[1], Axyz[2]) * 180.0 / M_PI;
    
    // Tilt-compensated heading (relative; no declination)
    heading = getHeading(Axyz, Mxyz, p);

    // Store (scaled where needed)
    moSbdMessage.pitch = (int16_t)lroundf(pitch * 100.0);
    moSbdMessage.roll = (int16_t)lroundf(roll * 100.0);
    moSbdMessage.heading = (uint16_t)heading;

  } else {
    DEBUG_PRINTLN("[IMU] Warning: LSM6DSOX + LIS3MDL offline!");
  }

  // Power down IMU
  disableImuPower();

  // Record elapsed execution time
  timer.readLsm6dsox = millis() - startTime;
}

// -------------------------------------------------------
// Helpers
// -------------------------------------------------------
static inline bool normalizeSafe(float v[3]) {
  float m2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
  if (!(m2 > 0.0f) || isnan(m2)) return false;
  float inv = 1.0 / sqrtf(m2);
  v[0] *= inv;
  v[1] *= inv;
  v[2] *= inv;
  return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

static inline void discardImuWarmupSamples(int n) {
  sensors_event_t accel, gyro, temp, magEvt;
  for (int i = 0; i < n; ++i) {
    if (online.lis3mdl) lis3mdl.getEvent(&magEvt);
    if (online.lsm6dsox) lsm6dsox.getEvent(&accel, &gyro, &temp);
    myDelay(10);  // simple fixed delay; not ODR-aligned by design
  }
}

// ----------------------------------------------------------------------------
// Returns a heading [0..359] (degrees given an acceleration vector a due to
// gravity (acc), magnetic vector m, and forward vector p.
// ----------------------------------------------------------------------------
int getHeading(float acc[3], float mag[3], float p[3]) {
  float W[3], N[3];  // West, North

  // cross(Up, mag) => West
  vectorCross(acc, mag, W);
  if (!normalizeSafe(W)) return 0;

  // cross(West, Up) => North
  vectorCross(W, acc, N);
  if (!normalizeSafe(N)) return 0;

  // Heading in horizontal plane (clockwise positive)
  float num = vectorDot(W, p);
  float den = vectorDot(N, p);
  int heading = (int)lroundf(atan2f(num, den) * 180.0 / (float)M_PI);
  heading = (heading + 360) % 360;
  return heading;
}

// Vector math
void vectorCross(float a[3], float b[3], float out[3]) {
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}
float vectorDot(float a[3], float b[3]) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
void vectorNormalize(float a[3]) {  // Kept for compatibility; prefer normalizeSafe
  float m = sqrt(vectorDot(a, a));
  if (m != 0) {
    a[0] /= m;
    a[1] /= m;
    a[2] /= m;
  }
}
