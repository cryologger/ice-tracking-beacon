// Configure IMU
void configureImu()
{
  DEBUG_PRINT("Info: Initializing IMU...");
  if (imu.init())
  {
    online.imu = true;

    //setLedColour(CRGB::Blue); // Change LED colour
    imu.enableDefault(); // Turn on accelerometer and magnetometer
    /*
      Calibration values: the default values of +/-32767 for each axis lead to an assumed
      magnetometer bias of 0. Use the Pololu LSM303 library Calibrate example program to
      determine appropriate values for each LSM303 sensor.
    */
    // 2 = 1, 1 = 3, 3 = 2
    imu.m_min = (LSM303::vector<int16_t>)
    {
      -32767, -32767, -32767
    };
    imu.m_max = (LSM303::vector<int16_t>)
    {
      +32767, +32767, +32767
    };
    DEBUG_PRINTLN("success!");
  }
  else
  {
    DEBUG_PRINTLN(F("failed!"));
    online.imu = false;
    //setLedColour(CRGB::Red); // Change LED colour
    blinkLed(3, 1000);
  }
}

// Read IMU
void readImu()
{
  // Start loop timer
  unsigned long loopStartTime = millis();

  // Check if IMU initialized successfully
  if (online.imu)
  {
    // Change LED colour
    //setLedColour(CRGB::Blue);

    DEBUG_PRINT("Info: Reading IMU...");

    // Average accelerometer readings
    float fXa, fYa, fZa = 0.0;
    float alpha = 0.10; // Alpha

    // Apply low-pass filter to accelerometer data
    for (byte i = 0; i < 30; i++)   // 30 samples
    {
      imu.read();   // Read accelerometer and magnetometer data
      fXa = imu.a.x * alpha + (fXa * (1.0 - alpha));
      fYa = imu.a.y * alpha + (fYa * (1.0 - alpha));
      fZa = imu.a.z * alpha + (fZa * (1.0 - alpha));
      delay(5);
    }

    // Write registers to put accelerometer and magenteometer in power-down mode
    imu.writeAccReg(0x20, 0x07);
    imu.writeMagReg(0x26, 0x03);

    // Calculate orientation
    float pitch = (atan2(-fXa, sqrt((int32_t)fYa * fYa + (int32_t)fZa * fZa))) * 180 / PI;
    float roll = (atan2(fYa, fZa)) * 180 / PI;
    float heading = imu.heading((LSM303::vector<int>)
    {
      1, 0, 0   // PCB orientation
    });

    // Write orientation data to union
    moSbdMessage.pitch = pitch * 100;
    moSbdMessage.roll = roll * 100;
    moSbdMessage.heading = heading * 10;

    DEBUG_PRINTLN("done.");

    //setLedColour(CRGB::Green); // Change LED colour
  }
  else
  {
    DEBUG_PRINTLN("Warning: IMU offline!");
    //setLedColour(CRGB::Red); // Change LED colour
  }
  // Stop loop timer
  timer.imu = millis() - loopStartTime;
}
