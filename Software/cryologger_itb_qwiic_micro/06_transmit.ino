// Configure RockBLOCK 9603
void configureIridium()
{
  //modem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE); // Assume USB power
  modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE); // Assume battery power
  modem.adjustATTimeout(20);            // Adjust timeout timer for serial AT commands (default = 20 s)
  modem.adjustSendReceiveTimeout(60);  // Adjust timeout timer for library send/receive commands (default = 300 s)
  online.iridium = true;
}

// Write data from structure to transmit buffer
void writeBuffer()
{
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
void transmitData()
{
  // Check if data transmission is required
  if ((online.iridium) && (transmitCounter == transmitInterval))
  {
    // Change LED colour
    setLedColour(purple);

    unsigned long loopStartTime = millis(); // Start loop timer

    // Start the serial port connected to the satellite modem
    IRIDIUM_PORT.begin(19200);

    // Begin satellite modem operation
    DEBUG_PRINTLN("Starting modem...");
    int err = modem.begin();
    if (err != ISBD_SUCCESS)
    {
      DEBUG_PRINT("Warning: Begin failed with error ");
      DEBUG_PRINTLN(err);
      if (err == ISBD_NO_MODEM_DETECTED)
      {
        DEBUG_PRINTLN("Warning: No modem detected! Check wiring.");
      }
      return;
    }
    /*
      // Test the signal quality
      int signalQuality = -1;
      err = modem.getSignalQuality(signalQuality);
      if (err != ISBD_SUCCESS) {
        DEBUG_PRINT("Warning: Signal quality failed with error ");
        DEBUG_PRINTLN(err);
        return;
      }
      DEBUG_PRINT("On a scale of 0 to 5, signal quality is currently: ");
      DEBUG_PRINTLN(signalQuality);
    */

    // Create buffer to store Mobile Terminated (MT) SBD message (270 bytes max)
    uint8_t mtBuffer[270];
    size_t mtBufferSize = sizeof(mtBuffer);
    memset(mtBuffer, 0x00, sizeof(mtBuffer)); // Clear mtBuffer array

    DEBUG_PRINTLN("Attempting to transmit message...");

    // Transmit and receieve SBD message data in binary format
    err = modem.sendReceiveSBDBinary(transmitBuffer, (sizeof(moMessage) * (transmitCounter + (retransmitCounter * transmitInterval))), mtBuffer, mtBufferSize);

    // Check if transmission was successful
    if (err != ISBD_SUCCESS)
    {
      DEBUG_PRINT("Warning: Transmission failed with error code ");
      DEBUG_PRINTLN(err);
      setLedColour(red); // Change LED colour to indicate failed transmission
    }
    else
    {
      DEBUG_PRINTLN("Transmission successful!");
      setLedColour(green); // Change LED colour to indicate a successful transmission

      retransmitCounter = 0; // Clear message retransmit counter
      memset(transmitBuffer, 0x00, sizeof(transmitBuffer)); // Clear transmit buffer array

      // Check if a Mobile Terminated (MT) message was received
      // If no message is available, mtBufferSize = 0
      if (mtBufferSize > 0)
      {
        DEBUG_PRINT("MT message received. Size: ");
        DEBUG_PRINT(mtBufferSize); DEBUG_PRINTLN(" bytes");

        // Print each incoming byte of data contained in the mtBuffer
        for (byte i = 0; i < mtBufferSize; i++)
        {
          DEBUG_PRINT("Address: "); DEBUG_PRINT(i);
          DEBUG_PRINT("\tValue: "); DEBUG_PRINTLN_HEX(mtBuffer[i]);
        }

        // Recompose bits using bitshift
        uint8_t  resetFlagBuffer            = (((uint8_t)mtBuffer[8] << 0) & 0xFF);
        uint16_t retransmitCounterMaxBuffer = (((uint16_t)mtBuffer[7] << 0) & 0xFF) +
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
            (retransmitCounterMaxBuffer >= 0    && retransmitCounterMaxBuffer <= 24) &&
            (resetFlagBuffer            == 0    || resetFlagBuffer            == 255))
        {

          // Update global variables
          alarmInterval         = alarmIntervalBuffer;        // Update alarm interval
          transmitInterval      = transmitIntervalBuffer;     // Update transmit interval
          retransmitCounterMax  = retransmitCounterMaxBuffer; // Update max retransmit counter
          resetFlag             = resetFlagBuffer;            // Update force reset flag
        }
      }
    }

    // Clear the Mobile Originated message buffer
    DEBUG_PRINTLN("Clearing the MO buffer...");
    err = modem.clearBuffers(ISBD_CLEAR_MO); // Clear MO buffer
    if (err != ISBD_SUCCESS)
    {
      DEBUG_PRINT("Warning: Clear buffer failed with error ");
      DEBUG_PRINTLN(err);

      // Change LED colour
      setLedColour(orange); // Change LED colour to indicate failure
    }


    // Store message in transmit buffer if transmission or modem begin fails
    if (err != ISBD_SUCCESS)
    {
      retransmitCounter++;
      // Reset counter if reattempt limit is exceeded
      if (retransmitCounter >= retransmitCounterMax)
      {
        retransmitCounter = 0;
        memset(transmitBuffer, 0x00, sizeof(transmitBuffer)); // Clear transmitBuffer array
      }
      setLedColour(red);
    }

    // Put modem to sleep
    DEBUG_PRINTLN("Putting modem to sleep...");
    err = modem.sleep();
    if (err != ISBD_SUCCESS)
    {
      DEBUG_PRINT("Warning: Sleep failed error ");
      DEBUG_PRINTLN(err);
      setLedColour(orange); // Change LED colour to indicate failure to sleep
    }

    // Close the serial port connected to the RockBLOCK
    IRIDIUM_PORT.end();

    transmitCounter = 0;  // Reset transmit counter
    unsigned long loopEndTime = millis() - loopStartTime; // Stop loop timer
    moMessage.transmitDuration = loopEndTime / 1000;

    DEBUG_PRINT("transmitData() function execution: "); DEBUG_PRINT(loopEndTime); DEBUG_PRINTLN(" ms");
    DEBUG_PRINT("retransmitCounter: "); DEBUG_PRINTLN(retransmitCounter);

    // Check if reset flag was transmitted
    if (resetFlag)
    {
      DEBUG_PRINTLN("Forced system reset...");
      digitalWrite(LED_BUILTIN, HIGH); // Turn on LED
      while (true); // Wait for Watchdog Timer to reset system
    }
  }
  else
  {
    DEBUG_PRINTLN("Warning: Iridium 9603 not online!");
  }
}

// RockBLOCK callback function can be repeatedly called during transmission or GNSS signal acquisition
bool ISBDCallback()
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > 1000)
  {
    previousMillis = currentMillis;
    petDog();  // Reset the Watchdog Timer

    // Blink LED
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
  return true;
}

#if DEBUG_IRIDIUM
// Callback to sniff the conversation with the Iridium modem
void ISBDConsoleCallback(IridiumSBD *device, char c)
{
  DEBUG_WRITE(c);
}

// Callback to to monitor Iridium modem library's run state
void ISBDDiagsCallback(IridiumSBD *device, char c)
{
  DEBUG_WRITE(c);
}
#endif

// Call user function 1
void userFunction1()
{

}

// Call user function 2
void userFunction2()
{

}

// Call user function 3
void userFunction3()
{

}

// Call user function 4
void userFunction4()
{

}
