// Configure SparkFun ICM-20948
void configureImu() {

  imu.begin(Wire, 1);
  if (imu.status != ICM_20948_Stat_Ok ) {
    DEBUG_PRINTLN("Warning: ICM-20948 not detected at default I2C address. Please check wiring.");
    online.imu = false;
  }
  else {
    online.imu = true;
  }
}

// Read SparkFun ICM-20948
void readImu() {

  unsigned long loopStartTime = millis(); // Start loop timer
  
  setLedColour(magenta);

  if (online.imu) {
    // Wake the sensor
    imu.sleep(false);
    imu.lowPower(false);
    imu.getAGMT(); // Values are only updated when 'getAGMT' is called
    
#if DEBUG_IMU
    DEBUG_PRINT("Scaled. Acc (mg) [ ");
    DEBUG_PRINT_DEC(imu.accX(), 2);
    DEBUG_PRINT(", ");
    DEBUG_PRINT_DEC(imu.accY(), 2);
    DEBUG_PRINT(", ");
    DEBUG_PRINT_DEC(imu.accZ(), 2);
    DEBUG_PRINT(" ], Gyr (DPS) [ ");
    DEBUG_PRINT_DEC(imu.gyrX(), 2);
    DEBUG_PRINT(", ");
    DEBUG_PRINT_DEC(imu.gyrY(), 2);
    DEBUG_PRINT(", ");
    DEBUG_PRINT_DEC(imu.gyrZ(), 2);
    DEBUG_PRINT(" ], Mag (uT) [ ");
    DEBUG_PRINT_DEC(imu.magX(), 2);
    DEBUG_PRINT(", ");
    DEBUG_PRINT_DEC(imu.magY(), 2);
    DEBUG_PRINT(", ");
    DEBUG_PRINT_DEC(imu.magZ(), 2);
    DEBUG_PRINT(" ], Tmp (C) [ ");
    DEBUG_PRINT_DEC(imu.temp(), 2);
    DEBUG_PRINT(" ]");
    DEBUG_PRINTLN();
#endif
    setLedColour(green);

    // Put the sensor to sleep
    imu.sleep(true);
    imu.lowPower(true);
  }

  unsigned long loopEndTime = millis() - loopStartTime; // Stop loop timer
  DEBUG_PRINT("readImu() function execution: "); DEBUG_PRINT(loopEndTime); DEBUG_PRINTLN(" ms");

}
