// Configure SparkFun Qwiic Iridium 9603N
void configureIridium() {

  if (modem.isConnected()) {
    modem.adjustATTimeout(15);          // Set AT timeout (Default = 20 seconds)
    modem.adjustSendReceiveTimeout(60); // Set send/receive timeout (Default = 300 seconds)
    modem.enable841lowPower(true);      // Enable ATtiny841 low-power mode
    online.iridium = true;
  }
  else {
    DEBUG_PRINTLN("Warning: Qwiic Iridium 9603N not detected! Please check wiring.");
    //while (1);
  }
}

// Transmit data using SparkFun Qwiic Iridium 9603N
void transmitData() {

  // Check if data can and should be transmitted
  if ((online.iridium) && (transmitCounter == transmitInterval)) {

    unsigned long loopStartTime = millis(); // Loop timer
    int err;

    DEBUG_PRINTLN("Enabling the supercapacitor charger...");
    modem.enableSuperCapCharger(true); // Enable the supercapacitor charger

    // Wait for supercapacitor charger PGOOD signal to go HIGH for up to 2 minutes
    while ((!modem.checkSuperCapCharger()) && millis() - loopStartTime < 2UL * 60UL * 1000UL) {
      DEBUG_PRINTLN("Waiting for supercapacitors to charge...");
      blinkLed(1, 1000);
      petDog();
    }
    DEBUG_PRINTLN("Supercapacitors charged!");

    modem.enable9603Npower(true); // Enable power to the Qwiic Iridium 9603N

    // Begin satellite modem operation
    DEBUG_PRINTLN("Starting modem...");
    err = modem.begin();
    if (err == ISBD_SUCCESS) {
      uint8_t mtBuffer[240];  // Buffer to store incoming transmission (240 byte limit)
      size_t mtBufferSize = sizeof(mtBuffer);
      memset(mtBuffer, 0x00, sizeof(mtBuffer)); // Clear inBuffer array

      /*
          // Test the signal quality
          int signalQuality = -1;
          err = modem.getSignalQuality(signalQuality);
          if (err != ISBD_SUCCESS) {
            DEBUG_PRINT("SignalQuality failed: error ");
            DEBUG_PRINTLN(err);
            return;
          }
          DEBUG_PRINT("On a scale of 0 to 5, signal quality is currently: ");
          DEBUG_PRINTLN(signalQuality);
      */

      // Transmit and receieve data in binary format
      err = modem.sendReceiveSBDBinary(transmitBuffer, (sizeof(moMessage) * (transmitCounter + (retransmitCounter * transmitInterval))), mtBuffer, mtBufferSize);

      // Check if transmission was successful
      if (err == ISBD_SUCCESS) {
        
        retransmitCounter = 0;
        memset(transmitBuffer, 0x00, sizeof(transmitBuffer)); // Clear transmit buffer array
      
        DEBUG_PRINTLN("Transmission successful!");

        // Check for incoming SBD Mobile Terminated (MT) message
        // If no inbound message is available, inBufferSize = 0
        if (mtBufferSize > 0) {

          // Print mtBuffer contents and write incoming message buffer to union/structure
          DEBUG_PRINT("Inbound buffer size is: "); DEBUG_PRINTLN(mtBufferSize);
          char tempData[240];
          DEBUG_PRINTLN("Byte\tHex");
          for (int i = 0; i < sizeof(mtBuffer); ++i) {
            mtMessage.bytes[i] = mtBuffer[i];
            sprintf(tempData, "%d\t0x%02X", i, mtBuffer[i]);
            DEBUG_PRINTLN(tempData);
          }
        }
      }
      else {
        DEBUG_PRINT("Transmission failed: error "); DEBUG_PRINTLN(err);
      }
    }
    else {
      DEBUG_PRINT("Begin failed: error"); DEBUG_PRINTLN(err);
      if (err == ISBD_NO_MODEM_DETECTED) {
        DEBUG_PRINTLN("Warning: Qwiic Iridium 9603N not detected. Please check wiring.");
      }
      return;
    }

    // Store message in transmit buffer if transmission or modem begin fails
    if (err != ISBD_SUCCESS) {
      retransmitCounter++;
      // Reset counter if reattempt limit is exceeded
      if (retransmitCounter >= retransmitCounterMax) {
        retransmitCounter = 0;
        memset(transmitBuffer, 0x00, sizeof(transmitBuffer)); // Clear transmitBuffer array
      }
    }

    // Power down the modem
    DEBUG_PRINTLN("Putting the Qwiic Iridium 9603N to sleep.");
    err = modem.sleep();
    if (err != ISBD_SUCCESS) {
      DEBUG_PRINT("Sleep failed: error "); DEBUG_PRINTLN(err);
    }

    modem.enable9603Npower(false);      // Disable 9603N power
    modem.enableSuperCapCharger(false); // Disable the supercapacitor charger
    modem.enable841lowPower(true);      // Enable the ATtiny841 low power mode

    transmitCounter = 0;  // Reset transmit counter
    unsigned long loopEndTime = millis() - loopStartTime;
    moMessage.transmitDuration = loopEndTime / 1000;


    DEBUG_PRINT("transmitDuration: "); DEBUG_PRINTLN(loopEndTime / 1000);
    DEBUG_PRINT("retransmitCounter: "); DEBUG_PRINTLN(retransmitCounter);

    DEBUG_PRINT("transmitData() function execution: "); DEBUG_PRINT(loopEndTime); DEBUG_PRINTLN(" ms");
  }
}

// RockBLOCK callback function can be repeatedly called during transmission or GNSS signal acquisition
bool ISBDCallback() {

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > 1000) {
    previousMillis = currentMillis;
    petDog();  // Reset the Watchdog Timer

    // Blink LED

    ledStateFlag = !ledStateFlag; // Set LED state opposite to what it was previously
    digitalWrite(LED_BUILTIN, !ledStateFlag);
  }
  return true;
}

#if DEBUG_IRIDUM
// Callback to sniff the conversation with the Iridium modem
void ISBDConsoleCallback(IridiumSBD *device, char c) {
  DEBUG_WRITE(c);
}

// Callback to to monitor Iridium modem library's run state
void ISBDDiagsCallback(IridiumSBD *device, char c) {
  DEBUG_WRITE(c);
}
#endif

// Write data to transmit buffer
void writeBuffer() {

  messageCounter++;                           // Increment message counter
  moMessage.messageCounter = messageCounter;  // Write message counter data to union
  transmitCounter++;                          // Increment data transmission counter

  // Concatenate current message with existing message(s) stored in transmit buffer
  memcpy(transmitBuffer + (sizeof(moMessage) * (transmitCounter + (retransmitCounter * transmitInterval) - 1)), moMessage.bytes, sizeof(moMessage));

  printUnion();           // Print union
  //printUnionHex();        // Print union in hex
  //printTransmitBuffer();  // Print transmit buffer in hex
}
