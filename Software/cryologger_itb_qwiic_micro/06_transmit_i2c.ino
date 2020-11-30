// Configure SparkFun Qwiic Iridium 9603N
void configureIridiumI2C() {

  if (modem.isConnected()) {
    modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE); // Assume battery power
    modem.adjustATTimeout(30);          // Adjust timeout timer for serial AT commands (default = 20 s)
    modem.adjustSendReceiveTimeout(60); // Adjust timeout timer for library send/receive commands (default = 300 s)
    modem.enable841lowPower(true);      // Enable ATtiny841 low-power mode
    online.iridium = true;
  }
  else {
    SERIAL_PORT.println(F("Warning: Qwiic Iridium 9603N not detected! Please check wiring."));
    online.iridium = false;
  }
}

// Transmit data using SparkFun Qwiic Iridium 9603N
void transmitDataI2C() {

  setLedColour(purple);

  // Check if data can and should be transmitted
  if ((online.iridium) && (transmitCounter == transmitInterval)) {

    unsigned long loopStartTime = millis(); // Start loop timer
    int err;


    SERIAL_PORT.println(F("Disabling ATtiny841 low power mode..."));
    modem.enable841lowPower(false);

    SERIAL_PORT.println(F("Enabling the supercapacitor charger..."));
    modem.enableSuperCapCharger(true); // Enable the supercapacitor charger

    // Wait for supercapacitor charger PGOOD signal to go HIGH for up to 2 minutes
    SERIAL_PORT.println(F("Waiting for supercapacitors to charge..."));
    while (!modem.checkSuperCapCharger() && millis() - loopStartTime < 2UL * 60UL * 1000UL) {
      ISBDCallback();
    }
    SERIAL_PORT.println(F("Supercapacitors charged!"));

    // Enable power for the Qwiic Iridium 9603N
    SERIAL_PORT.println(F("Enabling Qwiic Iridium 9603N power..."));
    modem.enable9603Npower(true);

    // Begin satellite modem operation
    SERIAL_PORT.println(F("Starting modem..."));
    err = modem.begin();
    if (err == ISBD_SUCCESS) {
      uint8_t mtBuffer[270];  // Buffer to store incoming transmission (270-byte limit)
      size_t mtBufferSize = sizeof(mtBuffer);
      memset(mtBuffer, 0x00, sizeof(mtBuffer)); // Clear mtBuffer array

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
      err = modem.sendReceiveSBDBinary(transmitBuffer, (sizeof(message) * (transmitCounter + (retransmitCounter * transmitInterval))), mtBuffer, mtBufferSize);

      // Check if transmission was successful
      if (err == ISBD_SUCCESS) {
        retransmitCounter = 0;
        memset(transmitBuffer, 0x00, sizeof(transmitBuffer)); // Clear transmit buffer array
        SERIAL_PORT.println(F("Transmission successful!"));

        // Check for incoming message
        // If no inbound message is available mtBufferSize will be zero
        if (mtBufferSize > 0) {

          // Print mtBuffer size and values of each incoming byte of data
          SERIAL_PORT.print(F("Inbound buffer size is: ")); SERIAL_PORT.println(mtBufferSize);
          for (uint8_t i = 0; i < mtBufferSize; i++) {
            SERIAL_PORT.print(F("Address: ")); SERIAL_PORT.print(i);
            SERIAL_PORT.print(F("\tValue: ")); SERIAL_PORT.println(mtBuffer[i], HEX);
          }

          // Recompose bits using bitshift
          uint8_t  resetFlagBuffer            = (((uint8_t)mtBuffer[8] << 0) & 0xFF);
          uint16_t maxRetransmitCounterBuffer = (((uint16_t)mtBuffer[7] << 0) & 0xFF) +
                                                (((uint16_t)mtBuffer[6] << 8) & 0xFFFF);
          uint16_t transmitIntervalBuffer     = (((uint16_t)mtBuffer[5] << 0) & 0xFF) +
                                                (((uint16_t)mtBuffer[4] << 8) & 0xFFFF);
          uint32_t alarmIntervalBuffer        = (((uint32_t)mtBuffer[3] << 0) & 0xFF) +
                                                (((uint32_t)mtBuffer[2] << 8) & 0xFFFF) +
                                                (((uint32_t)mtBuffer[1] << 16) & 0xFFFFFF) +
                                                (((uint32_t)mtBuffer[0] << 24) & 0xFFFFFFFF);

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
    }

    // Power down the modem
    SERIAL_PORT.println(F("Putting the Qwiic Iridium 9603N to sleep..."));
    err = modem.sleep();
    if (err != ISBD_SUCCESS) {
      SERIAL_PORT.print(F("Sleep failed: error ")); SERIAL_PORT.println(err);
    }

    // Disable 9603N power
    SERIAL_PORT.println(F("Disabling 9603N power..."));
    modem.enable9603Npower(false);

    // Disable the supercapacitor charger
    SERIAL_PORT.println(F("Disabling the supercapacitor charger..."));
    modem.enableSuperCapCharger(false);

    // Enable the ATtiny841 low power mode
    SERIAL_PORT.println(F("Enabling ATtiny841 low power mode..."));
    modem.enable841lowPower(true); // Change this to false if you want to measure the current draw without enabling low power mode

    transmitCounter = 0;  // Reset transmit counter
    unsigned long loopEndTime = millis() - loopStartTime;
    message.transmitDuration = loopEndTime / 1000;

    SERIAL_PORT.print(F("transmitData() function execution: ")); SERIAL_PORT.print(loopEndTime); SERIAL_PORT.println(" ms");
    SERIAL_PORT.print(F("transmitDuration: ")); SERIAL_PORT.println(loopEndTime / 1000);
    SERIAL_PORT.print(F("retransmitCounter: ")); SERIAL_PORT.println(retransmitCounter);

    // Check if reset flag was transmitted
    if (resetFlag) {
      SERIAL_PORT.println(F("Forced system reset..."));
      digitalWrite(LED_BUILTIN, HIGH); // Turn on LED
      while (1); // Wait for Watchdog Timer to reset system
    }
  }
}
