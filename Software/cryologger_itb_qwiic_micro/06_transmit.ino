// Configure RockBLOCK 9603
void configureIridium() {

  //modem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE); // Assume battery power
  modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE); // Assume battery power
  modem.adjustATTimeout(30);            // Adjust timeout timer for serial AT commands (default = 20 s)
  modem.adjustSendReceiveTimeout(180);  // Adjust timeout timer for library send/receive commands (default = 300 s)
  online.iridium = true;
}

// Write data from structure to transmit buffer
void writeBuffer() {

  messageCounter++;                         // Increment message counter
  moMessage.messageCounter = messageCounter;  // Write message counter data to union
  transmitCounter++;                        // Increment data transmission counter

  // Concatenate current message with existing message(s) stored in transmit buffer
  memcpy(transmitBuffer + (sizeof(moMessage) * (transmitCounter + (retransmitCounter * transmitInterval) - 1)), moMessage.bytes, sizeof(moMessage));

  printUnion();
  //printUnionHex(); // Print union/structure in hex/binary
  //printTransmitBuffer();  // Print transmit buffer in hex/binary
}

// Transmit data using RockBLOCK 9603
void transmitData() {

  setLedColour(purple);

  // Check if data transmission is required
  if ((online.iridium) && (transmitCounter == transmitInterval)) {

    unsigned long loopStartTime = millis(); // Start loop timer
    int err;

    // Start the serial port connected to the satellite modem
    IRIDIUM_PORT.begin(19200);

    // Begin satellite modem operation
    DEBUG_PRINTLN("Starting modem...");
    err = modem.begin();
    if (err == ISBD_SUCCESS) {
      uint8_t mtBuffer[270];  // Buffer to store incoming transmission (MT SBD max message length: 270 bytes)
      size_t mtBufferSize = sizeof(mtBuffer);
      memset(mtBuffer, 0x00, sizeof(mtBuffer)); // Clear mtBuffer array

      /*
          // Test the signal quality
          int signalQuality = -1;
          err = modem.getSignalQuality(signalQuality);
          if (err != ISBD_SUCCESS) {
            DEBUG_PRINT("SignalQuality failed: error ");
            SERIAL_PORT.println(err);
            return;
          }
          DEBUG_PRINT("On a scale of 0 to 5, signal quality is currently: ");
          SERIAL_PORT.println(signalQuality);
      */

      // Transmit and receieve SBD message data in binary format
      err = modem.sendReceiveSBDBinary(transmitBuffer, (sizeof(moMessage) * (transmitCounter + (retransmitCounter * transmitInterval))), mtBuffer, mtBufferSize);

      // Check if transmission was successful
      if (err != ISBD_SUCCESS) {
        DEBUG_PRINT("Transmission failed with error code "); SERIAL_PORT.println(err);

        blinkLed(5, 1000); // Blink LED slowly to indicated failed transmission
        setLedColour(red); // Turn on LED to indicate failed transmission
      }
      else {
        DEBUG_PRINTLN("Transmission successful!");

        blinkLed(10, 100); // Blink LED quickly to indicate successful transmission
        setLedColour(green); // Turn on LED to indicate successful transmission

        retransmitCounter = 0; // Clear message retransmit counter
        memset(transmitBuffer, 0x00, sizeof(transmitBuffer)); // Clear transmit buffer array

        // Check if a Mobile Terminated (MT) message was received
        // If no message is available, mtBufferSize = 0
        if (mtBufferSize > 0) {

          DEBUG_PRINT("MT message received. Size: ");
          DEBUG_PRINT(mtBufferSize); DEBUG_PRINTLN(" bytes");

          // Print each incoming byte of data contained in the mtBuffer
          for (byte i = 0; i < mtBufferSize; i++) {
            DEBUG_PRINT("Address: "); DEBUG_PRINT(i);
            DEBUG_PRINT("\tValue: "); SERIAL_PORT.println(mtBuffer[i], HEX);
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

            // Update global variables
            alarmInterval         = alarmIntervalBuffer;        // Update alarm interval
            transmitInterval      = transmitIntervalBuffer;     // Update transmit interval
            maxRetransmitCounter  = maxRetransmitCounterBuffer; // Update max retransmit counter
            resetFlag             = resetFlagBuffer;            // Update force reset flag
          }
        }
      }

      // Clear the Mobile Originated message buffer
      DEBUG_PRINTLN("Clearing the MO buffer...");
      err = modem.clearBuffers(ISBD_CLEAR_MO); // Clear MO buffer
      if (err != ISBD_SUCCESS) {
        DEBUG_PRINT("Warning: modem.clearBuffers failed with error ");
        SERIAL_PORT.println(err);
        setLedColour(orange);
      }

    }
    else {
      DEBUG_PRINT("Begin failed: error "); SERIAL_PORT.println(err);
      if (err == ISBD_NO_MODEM_DETECTED) {
        DEBUG_PRINTLN("Warning: Qwiic Iridium 9603N not detected. Please check wiring.");
        setLedColour(red);
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
    DEBUG_PRINTLN("Putting the Qwiic Iridium 9603N to sleep...");
    err = modem.sleep();
    if (err != ISBD_SUCCESS) {
      DEBUG_PRINT("Sleep failed: error "); DEBUG_PRINTLN(err);
      setLedColour(orange);
    }

    // Close the serial port connected to the RockBLOCK
    IRIDIUM_PORT.end();

    transmitCounter = 0;  // Reset transmit counter
    unsigned long loopEndTime = millis() - loopStartTime; // Stop loop timer
    moMessage.transmitDuration = loopEndTime / 1000;

    DEBUG_PRINT("transmitData() function execution: "); DEBUG_PRINT(loopEndTime); DEBUG_PRINTLN(" ms");
    DEBUG_PRINT("retransmitCounter: "); DEBUG_PRINTLN(retransmitCounter);

    // Check if reset flag was transmitted
    if (resetFlag) {
      DEBUG_PRINTLN("Forced system reset...");
      digitalWrite(LED_BUILTIN, HIGH); // Turn on LED
      while (1); // Wait for Watchdog Timer to reset system
    }
  }
}

// RockBLOCK non-blocking callback function can be repeatedly called during transmission or GNSS signal acquisition
bool ISBDCallback() {
#if DEBUG_IRIDIUM
  digitalWrite(LED_BUILTIN, (millis() / 1000) % 2 == 1 ? HIGH : LOW);
#endif
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > 1000) {
    previousMillis = currentMillis;
    petDog(); // Reset the Watchdog Timer
    readBattery(); // Read battery voltage during transmission (when lowest voltage will be experienced)
  }
  return true;
}

#if DEBUG_IRIDIUM
// Callback to sniff the conversation with the Iridium modem
void ISBDConsoleCallback(IridiumSBD *device, char c) {
  SERIAL_PORT.write(c);
}

// Callback to to monitor Iridium modem library's run state
void ISBDDiagsCallback(IridiumSBD *device, char c) {
  SERIAL_PORT.write(c);
}
#endif



// Call user function 1
void userFunction1() {

}

// Call user function 2
void userFunction2() {

}

// Call user function 3
void userFunction3() {

}

// Call user function 4
void userFunction4() {

}
