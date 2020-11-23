// Configure SparkFun ICM-20948 IMU
void configureImu() {

  imu.begin(Wire, 1);
  if (imu.status != ICM_20948_Stat_Ok ) {
    SERIAL_PORT.println(F("Warning: ICM-20948 not detected at default I2C address. Please check wiring."));
    online.imu = false;
  }
  else {
    online.imu = true;
  }
}

// Read AHRS IMU
void readImu() {

  setPixelColour(cyan);

  // Start loop timer
  unsigned long loopStartTime = millis();

  if (online.imu) {
    // Wake the sensor
    imu.sleep(false);
    imu.lowPower(false);

    blinkLed(1, 100);

    imu.getAGMT(); // Values are only updated when 'getAGMT' is called
    SERIAL_PORT.print("Scaled. Acc (mg) [ ");
    SERIAL_PORT.print(imu.accX(), 2);
    SERIAL_PORT.print(", ");
    SERIAL_PORT.print(imu.accY(), 2);
    SERIAL_PORT.print(", ");
    SERIAL_PORT.print(imu.accZ(), 2);
    SERIAL_PORT.print(" ], Gyr (DPS) [ ");
    SERIAL_PORT.print(imu.gyrX(), 2);
    SERIAL_PORT.print(", ");
    SERIAL_PORT.print(imu.gyrY(), 2);
    SERIAL_PORT.print(", ");
    SERIAL_PORT.print(imu.gyrZ(), 2);
    SERIAL_PORT.print(" ], Mag (uT) [ ");
    SERIAL_PORT.print(imu.magX(), 2);
    SERIAL_PORT.print(", ");
    SERIAL_PORT.print(imu.magY(), 2);
    SERIAL_PORT.print(", ");
    SERIAL_PORT.print(imu.magZ(), 2);
    SERIAL_PORT.print(" ], Tmp (C) [ ");
    SERIAL_PORT.print(imu.temp(), 2);
    SERIAL_PORT.print(" ]");
    SERIAL_PORT.println();
    setPixelColour(green);

    // Put the sensor to sleep
    imu.sleep(true);
    imu.lowPower(true);
  }
  // Stop loop timer
  unsigned long loopEndTime = millis() - loopStartTime;
  SERIAL_PORT.print(F("readImu() function execution: ")); SERIAL_PORT.print(loopEndTime); SERIAL_PORT.println(F(" ms"));

}
