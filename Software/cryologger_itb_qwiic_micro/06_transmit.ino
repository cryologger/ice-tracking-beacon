// Configure SparkFun Qwiic Iridium 9603N
void configureIridium() {

  if (modem.isConnected()) {
    modem.adjustATTimeout(30);          // Set AT timeout (Default = 20 seconds)
    modem.adjustSendReceiveTimeout(60); // Set send/receive timeout (Default = 300 seconds)
    modem.enable841lowPower(true);      // Enable ATtiny841 low-power mode
    online.iridium = true;
  }
  else {
    Serial.println(F("Warning: Qwiic Iridium 9603N not detected! Please check wiring."));
    online.iridium = false;
  }
}

// Transmit data using SparkFun Qwiic Iridium 9603N
void transmitData() {

  // Check if data can and should be transmitted
  if ((online.iridium) && transmitCounter == transmitInterval)) {

    unsigned long loopStartTime = millis(); // Start loop timer
    int err;

    Serial.println(F("Disabling ATtiny841 low power mode..."));
    modem.enable841lowPower(false);

    Serial.println(F("Enabling the supercapacitor charger..."));
    modem.enableSuperCapCharger(true); // Enable the supercapacitor charger

    // Wait for supercapacitor charger PGOOD signal to go HIGH for up to 2 minutes
    while (!modem.checkSuperCapCharger() && millis() - loopStartTime < 2UL * 60UL * 1000UL) {
      Serial.println(F("Waiting for supercapacitors to charge..."));
      ISBDCallback();
    }
    Serial.println(F("Supercapacitors charged!"));

    // Enable power for the Qwiic Iridium 9603N
    Serial.println(F("Enabling Qwiic Iridium 9603N power..."));
    modem.enable9603Npower(true);

    // Begin satellite modem operation
    Serial.println(F("Starting modem..."));
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
            Serial.print(F("SignalQuality failed: error "));
            Serial.println(err);
            return;
          }
          Serial.print(F("On a scale of 0 to 5, signal quality is currently: "));
          Serial.println(signalQuality);
      */

      // Transmit and receieve data in binary format
      err = modem.sendReceiveSBDBinary(transmitBuffer, (sizeof(message) * (transmitCounter + (retransmitCounter * transmitInterval))), inBuffer, inBufferSize);

      // Check if transmission was successful
      if (err == ISBD_SUCCESS) {
        retransmitCounter = 0;
        memset(transmitBuffer, 0x00, sizeof(transmitBuffer)); // Clear transmit buffer array
        Serial.println(F("Transmission successful!"));

        // Check for incoming message
        // If no inbound message is available inBufferSize will be zero
        if (inBufferSize > 0) {

          // Print inBuffer size and values of each incoming byte of data
          Serial.print(F("Inbound buffer size is: ")); Serial.println(inBufferSize);
          for (uint8_t i = 0; i < inBufferSize; i++) {
            Serial.print(F("Address: ")); Serial.print(i);
            Serial.print(F("\tValue: ")); Serial.println(inBuffer[i], HEX);
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
        Serial.print(F("Transmission failed: error ")); Serial.println(err);
      }

    }
    else {
      Serial.print(F("Begin failed: error ")); Serial.println(err);
      if (err == ISBD_NO_MODEM_DETECTED) {
        Serial.println(F("Warning: Qwiic Iridium 9603N not detected. Please check wiring."));
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
    Serial.println(F("Putting the Qwiic Iridium 9603N to sleep..."));
    err = modem.sleep();
    if (err != ISBD_SUCCESS) {
      Serial.print(F("Sleep failed: error ")); Serial.println(err);
    }

    // Disable 9603N power
    Serial.println(F("Disabling 9603N power..."));
    modem.enable9603Npower(false);

    // Disable the supercapacitor charger
    Serial.println(F("Disabling the supercapacitor charger..."));
    modem.enableSuperCapCharger(false);

    // Enable the ATtiny841 low power mode
    Serial.println(F("Enabling ATtiny841 low power mode..."));
    modem.enable841lowPower(true); // Change this to false if you want to measure the current draw without enabling low power mode

    transmitCounter = 0;  // Reset transmit counter
    unsigned long loopEndTime = millis() - loopStartTime;
    message.transmitDuration = loopEndTime / 1000;

    Serial.print(F("transmitData() function execution: ")); Serial.print(loopEndTime); Serial.println(" ms");
    Serial.print(F("transmitDuration: ")); Serial.println(loopEndTime / 1000);
    Serial.print(F("retransmitCounter: ")); Serial.println(retransmitCounter);

    // Check if reset flag was transmitted
    if (resetFlag) {
      Serial.println(F("Forced system reset..."));
      digitalWrite(LED_BUILTIN, HIGH); // Turn on LED
      while (1); // Wait for Watchdog Timer to reset system
    }
  }
}

// RockBLOCK callback function can be repeatedly called during transmission or GNSS signal acquisition
bool ISBDCallback() {
#if DEBUG_IRIDIUM
  digitalWrite(LED_BUILTIN, (millis() / 1000) % 2 == 1 ? HIGH : LOW);
#endif
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > 1000) {
    previousMillis = currentMillis;
    petDog();  // Reset the Watchdog Timer
    readBattery(); // Read battery voltage during transmission (lowest voltage will be experienced)
  }
  return true;
}

// Callback to sniff the conversation with the Iridium modem
void ISBDConsoleCallback(IridiumSBD *device, char c) {
#if DEBUG_IRIDIUM
  Serial.write(c);
#endif
}

// Callback to to monitor Iridium modem library's run state
void ISBDDiagsCallback(IridiumSBD *device, char c) {
#if DEBUG_IRIDIUM
  Serial.write(c);
#endif
}

// Write data from structure to transmit buffer
void writeBuffer() {

  messageCounter++;                         // Increment message counter
  message.messageCounter = messageCounter;  // Write message counter data to union
  transmitCounter++;                        // Increment data transmission counter

  // Concatenate current message with existing message(s) stored in transmit buffer
  memcpy(transmitBuffer + (sizeof(message) * (transmitCounter + (retransmitCounter * transmitInterval) - 1)), message.bytes, sizeof(message));

#if DEBUG
  printUnion();
  printUnionBinary(); // Print union/structure in hex/binary
  //printTransmitBuffer();  // Print transmit buffer in hex/binary
#endif
}

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
