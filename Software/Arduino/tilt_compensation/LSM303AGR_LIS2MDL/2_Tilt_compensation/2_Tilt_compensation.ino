/*
  Title:    LSM303AGR + LIS2MDL Tilt-compensation Example
  Date:     October 21, 2022
  Author:   Adam Garbo

  Description:
  - Example to illustrate how to calculate pitch, roll and
  tilt-compensated heading from the Adafruit LSM303AGR + LIS2MDL
  - Based on Jim Remington's LSM9DS1 library: https://github.com/jremington/LSM9DS1-AHRS
  - Also based on Caleb Fenton's work: https://gist.github.com/CalebFenton/a97444750eb43e3354fd2d0196a2ebcf
  - Work in progress.

  From Jim:
  - The code assumes that the sensor board is oriented with Y pointing
  to the North, X pointing West, and Z pointing up.
  - The code compensates for tilts of up to 90 degrees away from horizontal.
  - Facing vector p is the direction of travel and allows reassigning these directions.
  - It should be defined as pointing forward, parallel to the ground,
  with coordinates {X, Y, Z} (in magnetometer frame of reference).

  From Caleb:
  - Returns the angular difference in the horizontal plane between the
  "from" vector and north, in degrees.
  Description of heading algorithm:
  - Shift and scale the magnetic reading based on calibration data to find
  the North vector. Use the acceleration readings to determine the Up
  vector (gravity is measured as an upward acceleration).
  - The cross product of North and Up vectors is East.
  - The vectors East and North form a basis for the horizontal plane.
  - The From vector is projected into the horizontal plane and the angle between
  the projected vector and horizontal north is returned.
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_LSM303_Accel.h>
#include <math.h>

Adafruit_LIS2MDL lis2mdl = Adafruit_LIS2MDL(12345);
Adafruit_LSM303_Accel_Unified lsm303agr = Adafruit_LSM303_Accel_Unified(54321);

/*
  Default vector as indicated by the silkscreen
  {1, 0, 0}    // Align to X+
  {-1, 0, 0}   // Align to X-
  {0, 1, 0}    // Align to Y+
  {0, -1, 0}   // Align to Y-
  {0, 0, 1}    // Align to Z+
  {0, 0, -1}   // Align to Z-
*/
float p[] = {0, -1, 0};  // X marking on sensor board points toward yaw = 0

// Min/max magnetometer values
float m_min[3] = {
  //0,0,0
  -78.90, -44.25, -63.00
  //-75.60, -62.25, -52.35
  //-71.55, -39.45, -61.35
};

float m_max[3] = {
  //0,0,0
  24.90, 52.95, 29.85
  //18.15, 45.00, 24.75
  //51.75, 72.60, 78.30
};



void setup()
{
  pinMode(A3, OUTPUT);
  digitalWrite(A3, HIGH);
  pinMode(A4, OUTPUT);
  digitalWrite(A4, HIGH);

  Serial.begin(115200);
  while (!Serial);
}

void loop()
{
  // Returns the angular difference in the horizontal plane between
  // a default vector and north, in degrees.
  readLsm303agr();
}

void configureLsm303agr()
{
  if (!lis2mdl.begin())
  {
    Serial.println("Warning: Unable to initialize magnetometer");
    while (1);
  }

  if (!lsm303agr.begin()) {
    Serial.println("Unable to initialize accelerometer");
    while (1);
  }
}

// Returns a set of scaled magnetic readings from the LIS3MDL
void readLis2mdl(float Mxyz[3])
{
  float temp[3];

  // Read magnetometer
  sensors_event_t event;
  lis2mdl.getEvent(&event);
  Mxyz[0] += event.magnetic.x;
  Mxyz[1] += event.magnetic.y;
  Mxyz[2] += event.magnetic.z;

  // Subtract average of min and max from magnetometer calibration
  Mxyz[0] -= (m_min[0] + m_max[0]) / 2;
  Mxyz[1] -= (m_min[1] + m_max[1]) / 2;
  Mxyz[2] -= (m_min[2] + m_max[2]) / 2;
}

void readLsm303agr()
{
  // Initialize IMU
  configureLsm303agr();

  // Centered and scaled accelerometer/magnetomer data initialized to zero
  float Axyz[3] = {};
  float Mxyz[3] = {};
  int heading = 0;

  // Read scaled magnetometer data
  readLis2mdl(Mxyz);

  // Read normalized accelerometer data
  sensors_event_t event;
  lsm303agr.getEvent(&event);
  Axyz[0] += event.acceleration.x;
  Axyz[1] += event.acceleration.y;
  Axyz[2] += event.acceleration.z;

  // Calculate pitch, roll and tilt-compensated heading
  float pitch = atan2(-Axyz[0], sqrt(Axyz[1] * Axyz[1] + Axyz[2] * Axyz[2])) * 180 / PI;
  float roll = atan2(Axyz[1], Axyz[2]) * 180 / PI;

  for (byte i = 0; i < 3; i++)
  {
    heading = getHeading(Axyz, Mxyz, p);
    delay(100);
  }
  Serial.print(pitch); Serial.print(","); Serial.print(roll); Serial.print(","); Serial.println(heading);
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
