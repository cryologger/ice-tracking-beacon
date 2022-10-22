/*
  Title:    LSM303AGR + LIS2MDL Calibration
  Date:     October 21, 2022
  Author:   Adam Garbo

  Description:
  - Records the minimum and maximum readings of the LIS2MDL magnetometer for each axis (x,y,z)
  - Slowly rotate the LSM303AGR module repeatedly along all three axes
  - The values displayed will be the the min and max ranges for each axis and 
  can be used to re-scale the output of the sensor.
  - Values obtained via the calibration sketch can be used to perform a 2-point calibration 
  on each of the 3 axis
  - This method does *not* account for hard or soft iron effects
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS2MDL.h>

Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345);

float magMinX, magMaxX;
float magMinY, magMaxY;
float magMinZ, magMaxZ;

unsigned long previousMillis = 0;

void setup()
{
  pinMode(A4, OUTPUT);
  digitalWrite(A4, HIGH);

  Serial.begin(115200);
  Serial.println("LIS2MDL Calibration"); Serial.println("");
  while (!Serial);

  // Initialise the magnetometer
  if (!mag.begin())
  {
    // There was a problem detecting the LIS2MDL ... check your connections
    Serial.println("Ooops, no LIS2MDL detected ... Check your wiring!");
    while (1);
  }
  previousMillis = millis();
}

void loop()
{
  // Get a new sensor event
  sensors_event_t event;
  mag.getEvent(&event);

  if (event.magnetic.x < magMinX) magMinX = event.magnetic.x;
  if (event.magnetic.x > magMaxX) magMaxX = event.magnetic.x;

  if (event.magnetic.y < magMinY) magMinY = event.magnetic.y;
  if (event.magnetic.y > magMaxY) magMaxY = event.magnetic.y;

  if (event.magnetic.z < magMinZ) magMinZ = event.magnetic.z;
  if (event.magnetic.z > magMaxZ) magMaxZ = event.magnetic.z;

  if ((millis() - previousMillis) > 500)
  {
    Serial.print("max: {"); Serial.print(magMaxX); Serial.print(", "); Serial.print(magMaxY); Serial.print(", "); Serial.print(magMaxZ); Serial.println("}");
    Serial.print("min: {"); Serial.print(magMinX); Serial.print(", "); Serial.print(magMinY); Serial.print(", "); Serial.print(magMinZ); Serial.println("}");

    previousMillis = millis();
  }
}
