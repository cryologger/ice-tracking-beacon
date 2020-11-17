void configureImu() {
  imu.begin(Wire, 1);
  if (imu.status != ICM_20948_Stat_Ok ) {
    Serial.println(F("Warning: ICM-20948 not detected at default I2C address. Please check wiring."));
    online.imu = false;
  } else {
    online.imu = true;
  }
}

void readImu() {

  setPixelColour(cyan);

  unsigned long loopStartTime = millis(); // Loop timer

  // Wake the sensor
  imu.sleep(false);
  imu.lowPower(false);

  while (!imu.dataReady() && millis() - loopStartTime < 1UL * 10UL * 1000UL) {
    blinkLed(1, 1000);
    if (imu.dataReady()) {
      imu.getAGMT(); // Values are only updated when 'getAGMT' is called
      Serial.print("Scaled. Acc (mg) [ ");
      Serial.print(imu.accX(), 2);
      Serial.print(", ");
      Serial.print(imu.accY(), 2);
      Serial.print(", ");
      Serial.print(imu.accZ(), 2);
      Serial.print(" ], Gyr (DPS) [ ");
      Serial.print(imu.gyrX(), 2);
      Serial.print(", ");
      Serial.print(imu.gyrY(), 2);
      Serial.print(", ");
      Serial.print(imu.gyrZ(), 2);
      Serial.print(" ], Mag (uT) [ ");
      Serial.print(imu.magX(), 2);
      Serial.print(", ");
      Serial.print(imu.magY(), 2);
      Serial.print(", ");
      Serial.print(imu.magZ(), 2);
      Serial.print(" ], Tmp (C) [ ");
      Serial.print(imu.temp(), 2);
      Serial.print(" ]");
      Serial.println();
      setPixelColour(green);
    }
  }
  if (!imu.dataReady()) {
    setPixelColour(red);
  }

  imu.sleep(true);
  imu.lowPower(true);

  unsigned long loopEndTime = millis() - loopStartTime;
  Serial.print(F("readImu() function execution: ")); Serial.print(loopEndTime); Serial.println(F(" ms"));

}
