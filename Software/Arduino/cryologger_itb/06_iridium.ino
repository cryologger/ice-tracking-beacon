/*
  Iridium Module

  This module configures and manages the RockBLOCK 9603 Iridium modem.
  It provides functions to set up the modem, build and transmit SBD messages,
  handle incoming MT messages, and manage power states. Callbacks for the
  IridiumSBD library are included for debugging and non-blocking transmission
  or GNSS acquisition.
*/

// ----------------------------------------------------------------------------
// Configure RockBLOCK 9603.
// ----------------------------------------------------------------------------
void configureIridium() {
  // Configure for battery power
  // For USB power use: IridiumSBD::USB_POWER_PROFILE)
  modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE);

  // Timeout for Iridium send/receive commands (default = 300 s)
  modem.adjustSendReceiveTimeout(iridiumTimeout);

  // Timeout for Iridium startup (default = 240 s)
  modem.adjustStartupTimeout(IridiumStartup);

  DEBUG_PRINTLN("[Iridium] Info: RockBLOCK 9603 initialized.");
}

// ----------------------------------------------------------------------------
// Validate MT SBD message
// ----------------------------------------------------------------------------
bool validateMtSbdMessage(const SBD_MT_MESSAGE& msg) {
  return (
    msg.alarmMode <= 2
    && msg.alarmIntervalDay <= 31
    && msg.alarmIntervalHour >= 1 && msg.alarmIntervalHour < 24
    && msg.alarmIntervalMinute < 60
    && msg.transmitInterval >= 1 && msg.transmitInterval <= 24
    && msg.transmitReattempts <= 24
    && (msg.resetFlag == 0 || msg.resetFlag == 255));
}

// ----------------------------------------------------------------------------
// Write data from the MO-SBD structure to the transmit buffer.
// ----------------------------------------------------------------------------
void writeBuffer() {
  // Increment counters
  iterationCounter++;
  transmitCounter++;

  // Write message counter data to union
  moSbdMessage.iterationCounter = iterationCounter;

  // Concatenate current message with any existing messages in the buffer
  memcpy(
    moSbdBuffer + (sizeof(moSbdMessage) * (transmitCounter + (transmitReattemptCounter * transmitInterval) - 1)),
    moSbdMessage.bytes,
    sizeof(moSbdMessage));

  // Print MO-SBD structure data
  printMoSbd();
  printMoSbdHex();
  printMoSbdBuffer();

  // Clear the MO-SBD message structure after copying
  //memset(&moSbdMessage, 0x00, sizeof(moSbdMessage));
  moSbdMessage = {};
}

// ----------------------------------------------------------------------------
// Attempt to transmit data via the RockBLOCK 9603. Handles power enable,
// serial port initialization, message sending, reception, and power down.
// ----------------------------------------------------------------------------
void transmitData() {
  // Start execution timer.
  unsigned long startTime = millis();

  // Enable power to the RockBLOCK 9603
  enable5V();

  // Open the Iridium serial port at 19200 baud
  IRIDIUM_PORT.begin(19200);

  // Assign pins for SERCOM functionality for the new Serial2 instance
  pinPeripheral(PIN_IRIDIUM_TX, PIO_SERCOM);
  pinPeripheral(PIN_IRIDIUM_RX, PIO_SERCOM);

  // Wake up the modem and begin communications
  DEBUG_PRINTLN("[Iridium] Info: Starting modem...");
  int returnCode = modem.begin();

  if (returnCode != ISBD_SUCCESS) {
    DEBUG_PRINT("[Iridium] Warning: Begin failed with error ");
    DEBUG_PRINTLN(returnCode);
    if (returnCode == ISBD_NO_MODEM_DETECTED) {
      DEBUG_PRINTLN("[Iridium] Warning: No modem detected! Check wiring.");
    }
  } else {
    // Calculate SBD buffer sizes for MO and MT messages
    moSbdBufferSize = sizeof(moSbdMessage) * (transmitCounter + (transmitReattemptCounter * transmitInterval));
    mtSbdBufferSize = sizeof(mtSbdBuffer);

    // Clear MT-SBD buffer prior to receiving
    memset(mtSbdBuffer, 0x00, sizeof(mtSbdBuffer));

    DEBUG_PRINTLN("[Iridium] Info: Attempting to transmit message...");

    // Transmit/receive SBD message data in binary format
    returnCode = modem.sendReceiveSBDBinary(
      moSbdBuffer,
      moSbdBufferSize,
      mtSbdBuffer,
      mtSbdBufferSize);

    // Check if transmission was successful
    if (returnCode == ISBD_SUCCESS) {
      DEBUG_PRINTLN("[Iridium] Info: MO-SBD message transmission successful!");
      blinkLed(10, 250);

      // Clear counters and buffers
      failureCounter = 0;
      transmitReattemptCounter = 0;
      memset(moSbdBuffer, 0x00, sizeof(moSbdBuffer));

      // Check if a Mobile Terminated (MT) message was received
      // If no message is available, mtSbdBufferSize = 0
      if (mtSbdBufferSize > 0) {
        DEBUG_PRINT("[Iridium] Info: MT-SBD message received. Size = ");
        DEBUG_PRINT(mtSbdBufferSize);
        DEBUG_PRINTLN(" bytes.");

        // Check if MT-SBD message is the correct size (7 bytes).
        if (mtSbdBufferSize == 7) {
          DEBUG_PRINTLN("[Iridium] Info: MT-SBD message correct size.");

          // Copy incoming data into the MT-SBD structure.
          for (size_t i = 0; i < mtSbdBufferSize; ++i) {
            mtSbdMessage.bytes[i] = mtSbdBuffer[i];
          }

          // Print the MT-SBD message
          printMtSbdBuffer();  // Print MT-SBD message in hexadecimal
          printMtSbd();        // Print MT-SBD message stored in union/structure

          if (validateMtSbdMessage(mtSbdMessage)) {
            DEBUG_PRINTLN("[Iridium] Info: All received values within accepted ranges.");

            // Update config values from MT message
            transmitInterval = mtSbdMessage.transmitInterval;
            transmitReattempts = mtSbdMessage.transmitReattempts;
            resetFlag = mtSbdMessage.resetFlag;
            alarmMode = mtSbdMessage.alarmMode;
            alarmIntervalDay = mtSbdMessage.alarmIntervalDay;
            alarmIntervalHour = mtSbdMessage.alarmIntervalHour;
            alarmIntervalMinute = mtSbdMessage.alarmIntervalMinute;
          } else {
            DEBUG_PRINTLN("[Iridium] Warning: Received values exceed accepted range!");
          }

        } else {
          DEBUG_PRINTLN("[Iridium] Warning: MT-SBD message incorrect size!");
        }
      }
    } else {
      DEBUG_PRINT("[Iridium] Warning: Transmission failed with error code ");
      blinkLed(5, 1000);
      DEBUG_PRINTLN(returnCode);
    }
  }

  // Store return code in the MO-SBD structure for reference
  moSbdMessage.transmitStatus = returnCode;

  // If transmission or modem begin fails, increment counters and
  // potentially clear the buffer if reattempt limit is exceeded
  if (returnCode != ISBD_SUCCESS) {
    transmitReattemptCounter++;
    failureCounter++;

    if (transmitReattemptCounter > transmitReattempts) {
      transmitReattemptCounter = 0;
      memset(moSbdBuffer, 0x00, sizeof(moSbdBuffer));
    }
  }

  // Clear transmit buffer if program is running for the first time
  if (firstTimeFlag) {
    transmitReattemptCounter = 0;
    memset(moSbdBuffer, 0x00, sizeof(moSbdBuffer));
  }

  // Put the modem to sleep.
  DEBUG_PRINTLN("[Iridium] Info: Putting modem to sleep...");
  returnCode = modem.sleep();
  if (returnCode != ISBD_SUCCESS) {
    DEBUG_PRINT("[Iridium] Warning: Sleep failed error ");
    DEBUG_PRINTLN(returnCode);
  }

  // Close the serial port
  IRIDIUM_PORT.end();

  // Disable power to the RockBLOCK 9603
  disable5V();

  // Reset transmit counter after the attempt
  transmitCounter = 0;

  // Record elapsed execution time
  timer.iridium = millis() - startTime;

  // Write duration of the last transmission (in seconds) to the MO-SBD struct.
  moSbdMessage.transmitDuration = timer.iridium / 1000;

  // Print current settings to debug console
  printSettings();

  // Check if a forced reset was requested via the MT message
  if (resetFlag) {
    DEBUG_PRINTLN("[Iridium] Info: Forced system reset...");
    digitalWrite(LED_BUILTIN, HIGH);  // Turn on LED
    while (true)
      ;  // Wait for the Watchdog Timer to reset the system
  }
}

// ----------------------------------------------------------------------------
// Non-blocking callback function invoked during Iridium transmissions or
// GNSS signal acquisition. Here, we reset the watchdog periodically and
// optionally blink an LED to show activity.
// ----------------------------------------------------------------------------
bool ISBDCallback() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > 1000) {
    previousMillis = currentMillis;
    resetWdt();
    // readBattery();  // Optionally measure battery voltage during Iridium TX
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
  return true;
}

#if DEBUG_IRIDIUM
// ----------------------------------------------------------------------------
// Sniffs the conversation with the Iridium modem for debugging purposes.
// ----------------------------------------------------------------------------
void ISBDConsoleCallback(IridiumSBD* /*device*/, char c) {
  DEBUG_WRITE(c);
}

// ----------------------------------------------------------------------------
// Monitors the Iridium modem library's run state for debugging purposes.
// ----------------------------------------------------------------------------
void ISBDDiagsCallback(IridiumSBD* /*device*/, char c) {
  DEBUG_WRITE(c);
}
#endif

// ----------------------------------------------------------------------------
// Configure sleep pin.
// Overrides IridiumSBD::setSleepPin to handle the N-MOSFET inversion.
// When 'enable' == HIGH, we drive the pin LOW = awake. When 'enable' == LOW,
// we drive pin HIGH = asleep. The diagprint call prints the new state.
// ----------------------------------------------------------------------------
void IridiumSBD::setSleepPin(uint8_t enable) {
  if (enable == HIGH) {
    digitalWrite(this->sleepPin, LOW);  // LOW = awake (inverted by N-MOSFET)
    diagprint(F("AWAKE\r\n"));
  } else {
    digitalWrite(this->sleepPin, HIGH);  // HIGH = asleep (inverted by N-MOSFET)
    diagprint(F("ASLEEP\r\n"));
  }
}
