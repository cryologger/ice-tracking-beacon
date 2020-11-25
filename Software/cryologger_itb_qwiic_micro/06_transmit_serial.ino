// Configure RockBLOCK 9603
void configureIridiumSerial() {
  modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE); // Assume battery power
  //modem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE); // Assume USB power
  modem.adjustATTimeout(30);            // Adjust timeout timer for serial AT commands (default = 20 seconds)
  modem.adjustSendReceiveTimeout(180);  // Adjust timeout timer for library send/receive commands (default = 300 seconds)
  online.iridium = true;
}

// Transmit data using SparkFun Qwiic Iridium 9603N
void transmitDataSerial() {

  setLedColour(purple);

  // Check if data can and should be transmitted
  if ((online.iridium) && (transmitCounter == transmitInterval)) {

    unsigned long loopStartTime = millis(); // Start loop timer
    int err;

    // Start the serial port connected to the satellite modem
    IRIDIUM_PORT.begin(19200);

    // Begin satellite modem operation
    SERIAL_PORT.println(F("Starting modem..."));
    err = modem.begin();
    if (err == ISBD_SUCCESS) {
      uint8_t inBuffer[240];  // Buffer to store incoming transmission (240 byte limit)
      size_t inBufferSize = sizeof(inBuffer);
      memset(inBuffer, 0x00, sizeof(inBuffer)); // Clear inBuffer array

      /*
          // Test the signal quality
          int signalQuality = -1;
          err = modem.getSignalQuality(signalQuality);
          if (err != ISBD_SUCCESS) {
            SERIAL_PORT.print(F("SignalQuality failed: error "));
            SERIAL_PORT.println(err);
            return;
          }
          SERIAL_PORT.print(F("On a scale of 0 to 5, signal quality is currently: "));
          SERIAL_PORT.println(signalQuality);
      */

      // Transmit and receieve data in binary format
      err = modem.sendReceiveSBDBinary(transmitBuffer, (sizeof(message) * (transmitCounter + (retransmitCounter * transmitInterval))), inBuffer, inBufferSize);

      // Check if transmission was successful
      if (err == ISBD_SUCCESS) {
        setLedColour(green);
        retransmitCounter = 0;
        memset(transmitBuffer, 0x00, sizeof(transmitBuffer)); // Clear transmit buffer array
        SERIAL_PORT.println(F("Transmission successful!"));

        // Check for incoming message
        // If no inbound message is available inBufferSize will be zero
        if (inBufferSize > 0) {

          // Print inBuffer size and values of each incoming byte of data
          SERIAL_PORT.print(F("Inbound buffer size is: ")); SERIAL_PORT.println(inBufferSize);
          for (uint8_t i = 0; i < inBufferSize; i++) {
            SERIAL_PORT.print(F("Address: ")); SERIAL_PORT.print(i);
            SERIAL_PORT.print(F("\tValue: ")); SERIAL_PORT.println(inBuffer[i], HEX);
          }

          // Recompose bits using bitshift
          uint8_t  resetFlagBuffer            = (((uint8_t)inBuffer[8] << 0) & 0xFF);
          uint16_t maxRetransmitCounterBuffer = (((uint16_t)inBuffer[7] << 0) & 0xFF) +
                                                (((uint16_t)inBuffer[6] << 8) & 0xFFFF);
          uint16_t transmitIntervalBuffer     = (((uint16_t)inBuffer[5] << 0) & 0xFF) +
                                                (((uint16_t)inBuffer[4] << 8) & 0xFFFF);
          uint32_t alarmIntervalBuffer        = (((uint32_t)inBuffer[3] << 0) & 0xFF) +
                                                (((uint32_t)inBuffer[2] << 8) & 0xFFFF) +
                                                (((uint32_t)inBuffer[1] << 16) & 0xFFFFFF) +
                                                (((uint32_t)inBuffer[0] << 24) & 0xFFFFFFFF);

          // Check if incoming data is valid
          if ((alarmIntervalBuffer        >= 300  && alarmIntervalBuffer        <= 1209600) &&
              (transmitIntervalBuffer     >= 1    && transmitIntervalBuffer     <= 24) &&
              (maxRetransmitCounterBuffer >= 0    && maxRetransmitCounterBuffer <= 24) &&
              (resetFlagBuffer            == 0    || resetFlagBuffer            == 255)) {

            // Update variables
            alarmInterval         = alarmIntervalBuffer;        // Update alarm interval
            transmitInterval      = transmitIntervalBuffer;     // Update transmit interval
            maxRetransmitCounter  = maxRetransmitCounterBuffer; // Update max retransmit counter
            resetFlag             = resetFlagBuffer;            // Update force reset flag
          }
        }
      }
      else {
        SERIAL_PORT.print(F("Transmission failed: error ")); SERIAL_PORT.println(err);
      }

    }
    else {
      SERIAL_PORT.print(F("Begin failed: error ")); SERIAL_PORT.println(err);
      if (err == ISBD_NO_MODEM_DETECTED) {
        SERIAL_PORT.println(F("Warning: Qwiic Iridium 9603N not detected. Please check wiring."));
      }
      return;
    }

    // Store message in transmit buffer if transmission or modem begin fails
    if (err != ISBD_SUCCESS) {
      retransmitCounter++;
      // Reset counter if reattempt limit is exceeded
      if (retransmitCounter >= maxRetransmitCounter) {
        retransmitCounter = 0;
        memset(transmitBuffer, 0x00, sizeof(transmitBuffer)); // Clear transmitBuffer array
      }
      setLedColour(red);
    }

    // Power down the modem
    SERIAL_PORT.println(F("Putting the Qwiic Iridium 9603N to sleep..."));
    err = modem.sleep();
    if (err != ISBD_SUCCESS) {
      SERIAL_PORT.print(F("Sleep failed: error ")); SERIAL_PORT.println(err);
      setLedColour(orange);
    }

    // Close the serial port connected to the RockBLOCK modem
    IRIDIUM_PORT.end();

    transmitCounter = 0;  // Reset transmit counter
    unsigned long loopEndTime = millis() - loopStartTime; // Stop loop timer
    message.transmitDuration = loopEndTime / 1000;

    SERIAL_PORT.print(F("transmitData() function execution: ")); SERIAL_PORT.print(loopEndTime); SERIAL_PORT.println(" ms");
    SERIAL_PORT.print(F("retransmitCounter: ")); SERIAL_PORT.println(retransmitCounter);

    // Check if reset flag was transmitted
    if (resetFlag) {
      SERIAL_PORT.println(F("Forced system reset..."));
      digitalWrite(LED_BUILTIN, HIGH); // Turn on LED
      while (1); // Wait for Watchdog Timer to reset system
    }
  }
}
