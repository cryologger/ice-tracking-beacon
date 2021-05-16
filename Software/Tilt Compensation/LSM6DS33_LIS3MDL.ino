/*
  Example to illustrate how to calculate pitch, roll and 
  tilt-compensated heading from the Adafruit LSM6DS33 + LIS3MDL IMU.

  Date: May 16, 2021
  
  - The code assumes that the sensor board is oriented with Y pointing
  to the North, X pointing West, and Z pointing up.
  - The code compensates for tilts of up to 90 degrees away from horizontal.
  - Facing vector p is the direction of travel and allows reassigning these directions.
  - It should be defined as pointing forward, parallel to the ground, 
  with coordinates {X, Y, Z} (in magnetometer frame of reference).

  Code based on Jim Remington's LSM9DS1 library: https://github.com/jremington/LSM9DS1-AHRS

*/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>    // https://github.com/adafruit/Adafruit_Sensor
#include <Adafruit_LIS3MDL.h>   // https://github.com/adafruit/Adafruit_LIS3MDL
#include <Adafruit_LSM6DS33.h>  // https://github.com/adafruit/Adafruit_LSM6DS

Adafruit_LIS3MDL  lis3mdl;
Adafruit_LSM6DS33 lsm6ds33;

// Code initialization statements from Magneto required to correct magnetometer distortion
// See: https://forum.pololu.com/t/correcting-the-balboa-magnetometer/14315

float p[] = {1, 0, 0};  // Orientation of sensor

float M_B[3]
{ -2979.80,  432.81, -1757.11};

float M_Ainv[3][3]
{
  {  0.28665,  0.01375,  0.00138},
  {  0.01375,  0.28350, -0.00795},
  {  0.00138, -0.00795,  0.28812}
};

void setup()
{
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();
  if (!lis3mdl.begin_I2C())
  {
    Serial.println(F("Failed to initialize LIS3MDL!"));
  }

  if (!lsm6ds33.begin_I2C())
  {
    Serial.println(F("Failed to detect the LSM6."));
  }
}

void loop()
{
  float Axyz[3], Mxyz[3]; // Centered and scaled accel/mag data

  // Read scaled magnetometer data
  read_data(Mxyz);

  // Read accelerometer
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  lsm6ds33.getEvent(&accel, &gyro, &temp);

  // Normalize accelerometer data. Offsets and scale were determined to be negligible
  Axyz[0] = accel.acceleration.x;
  Axyz[1] = accel.acceleration.y;
  Axyz[2] = accel.acceleration.z;

  // Calculate pitch, roll and heading
  float pitch = atan2(-Axyz[0], sqrt(Axyz[1] * Axyz[1] + Axyz[2] * Axyz[2])) * 180 / PI;
  float roll = atan2(Axyz[1], Axyz[2]) * 180 / PI;
  int heading = get_heading(Axyz, Mxyz, p);

  Serial.print(pitch);
  Serial.print(", ");
  Serial.print(roll);
  Serial.print(", ");
  Serial.println(heading);

  delay(100);
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
