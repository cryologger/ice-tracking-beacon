/*
  Magnetometer Calibration Sampler (CSV)

  Collects raw LIS3MDL magnetometer data for offline 3×3 calibration.
  Output: "mx,my,mz" in microtesla (one sample per line).

  Instructions:
  - Open Serial Monitor/Plotter at 115200 baud.
  - When "START" prints, rotate the unit slowly through ALL orientations
    (big figure-eights, full sphere sweep), away from metal.
  - Copy the CSV (excluding lines starting with '#') to a file for Python/Magneto.

  Notes:
  - Uses Adafruit_LIS3MDL and Adafruit_LSM6DSOX.
*/

#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DSOX.h>

// -----------------------------------------------------------------------------
// Pin definitions
// -----------------------------------------------------------------------------
#define PIN_IMU_EN A4  // IMU digital pin enable

// -----------------------------------------------------------------------------
// Hardware objects
// -----------------------------------------------------------------------------
Adafruit_LIS3MDL lis3mdl;
Adafruit_LSM6DSOX lsm6dsox;

// -----------------------------------------------------------------------------
// Simple power/delay helpers (adapt if you control IMU power rails)
// -----------------------------------------------------------------------------
static inline void enableImuPower() { 
  digitalWrite(PIN_IMU_EN, HIGH);
}
static inline void disableImuPower() { 
  digitalWrite(PIN_IMU_EN, LOW);
}
static inline void myDelay(uint32_t ms) {
  delay(ms);
}

// -----------------------------------------------------------------------------
// Config
// -----------------------------------------------------------------------------
static const int CAL_WARMUP_SAMPLES = 20;    // Discard a handful on startup
static const int CAL_SAMPLE_DELAY_MS = 20;   // Short, fixed delay between samples
static const int CAL_SAMPLES_TARGET = 2400;  // Aim 500–2000 for a solid fit

// Run forever if true; otherwise stop after CAL_SAMPLES_TARGET
#define CAL_RUN_INDEFINITE false

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
void setup() {

  pinMode(PIN_IMU_EN, OUTPUT);

  enableImuPower();
  delay(5);

  Serial.begin(115200);
  while (!Serial) { /* wait for USB */
  }

  Wire.begin();

  // Init sensors (mirrors your project config where relevant)
  if (!lsm6dsox.begin_I2C()) {
    Serial.println(F("# ERROR: LSM6DSOX not detected"));
    hang();
  }
  lsm6dsox.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6dsox.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  lsm6dsox.setGyroDataRate(LSM6DS_RATE_SHUTDOWN);

  if (!lis3mdl.begin_I2C()) {
    Serial.println(F("# ERROR: LIS3MDL not detected"));
    hang();
  }
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_10_HZ);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);

  // Warm-up discard (simple fixed delay)
  sensors_event_t accel, gyro, temp, mag;
  for (int i = 0; i < CAL_WARMUP_SAMPLES; ++i) {
    lis3mdl.getEvent(&mag);
    lsm6dsox.getEvent(&accel, &gyro, &temp);
    myDelay(10);
  }

  // Header + instructions
  Serial.println(F("# LIS3MDL magnetometer calibration data"));
  Serial.println(F("# units: microtesla"));
  Serial.println(F("# columns: mx,my,mz"));
  Serial.println(F("# Rotate slowly through ALL orientations. START"));
  Serial.println(F("mx,my,mz"));
}

// -----------------------------------------------------------------------------
// Loop
// -----------------------------------------------------------------------------
void loop() {
  static int count = 0;
  sensors_event_t mag;
  lis3mdl.getEvent(&mag);

  // Print CSV (µT)
  Serial.print(mag.magnetic.x, 6);
  Serial.print(',');
  Serial.print(mag.magnetic.y, 6);
  Serial.print(',');
  Serial.println(mag.magnetic.z, 6);

  count++;
  if (!CAL_RUN_INDEFINITE && count >= CAL_SAMPLES_TARGET) {
    Serial.println(F("# END"));
    disableImuPower();
    hang();
  }

  myDelay(CAL_SAMPLE_DELAY_MS);
}

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------
[[noreturn]] void hang() {
  while (true) { myDelay(1000); }
}
