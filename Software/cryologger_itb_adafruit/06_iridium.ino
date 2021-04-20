// Configure RockBLOCK 9603
void configureIridium()
{
  //modem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE); // Assume USB power
  modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE); // Assume battery power
  modem.adjustATTimeout(30); // Adjust timeout timer for serial AT commands (default = 20 s)
  modem.adjustSendReceiveTimeout(iridiumTimeout); // Adjust timeout timer for library send/receive commands (default = 300 s)
  //modem.adjustStartupTimeout(30); // Adjust timeout for Iridium modem startup (default = 240 s)
}

// Write data from structure to transmit buffer
void writeBuffer()
{
  messageCounter++; // Increment message counter
  transmitCounter++; // Increment data transmission counter
  moMessage.messageCounter = messageCounter; // Write message counter data to union

  // Concatenate current message with existing message(s) stored in transmit buffer
  memcpy(transmitBuffer + (sizeof(moMessage) * (transmitCounter + (retransmitCounter * transmitInterval) - 1)), moMessage.bytes, sizeof(moMessage));

  // Print MO-SBD union/structure
  printMoSbd();
  //printMoSbdHex();
  //printTransmitBuffer();

  // Write zeroes to MO-SBD union/structure
  memset(&moMessage, 0, sizeof(moMessage));
}

// Attempt to transmit data via RockBLOCK 9603
void transmitData()
{
  // Start loop timer
  unsigned long loopStartTime = millis();

  // Check if data transmission interval has been reached
  if ((transmitCounter == transmitInterval) || firstTimeFlag)
  {
    // Change LED colour
    setLedColour(CRGB::Purple);

    // Start the serial port connected to the satellite modem
    IRIDIUM_PORT.begin(19200);

    // Assign pins for SERCOM functionality for new Serial2 instance
    pinPeripheral(PIN_IRIDIUM_TX, PIO_SERCOM);
    pinPeripheral(PIN_IRIDIUM_RX, PIO_SERCOM);

    // Wake up the RockBLOCK 9604 and prepare it to communicate
    DEBUG_PRINTLN("Info: Starting modem...");
    int returnCode = modem.begin();
    if (returnCode != ISBD_SUCCESS)
    {
      DEBUG_PRINT("Warning: Begin failed with error ");
      DEBUG_PRINTLN(returnCode);
      if (returnCode == ISBD_NO_MODEM_DETECTED)
      {
        DEBUG_PRINTLN("Warning: No modem detected! Check wiring.");
        setLedColourIridium(returnCode); // Set LED colour
      }
    }
    else
    {
      // Create buffer to store Mobile Terminated SBD (MT-SBD) message (270 bytes max)
      uint8_t mtBuffer[270];
      size_t mtBufferSize = sizeof(mtBuffer);
      memset(mtBuffer, 0x00, sizeof(mtBuffer)); // Clear mtBuffer array

      DEBUG_PRINTLN("Info: Attempting to transmit message...");

      // Transmit and receieve SBD message data in binary format
      returnCode = modem.sendReceiveSBDBinary(transmitBuffer, (sizeof(moMessage) * (transmitCounter + (retransmitCounter * transmitInterval))), mtBuffer, mtBufferSize);

      // Check if transmission was successful
      if (returnCode == ISBD_SUCCESS)
      {
        DEBUG_PRINTLN("Info: MO-SBD message transmission successful!");
        setLedColourIridium(returnCode); // Set LED colour to appropriate return code

        failedTransmitCounter = 0; // Clear failed transmission counter
        retransmitCounter = 0; // Clear message retransmit counter
        memset(transmitBuffer, 0x00, sizeof(transmitBuffer)); // Clear transmit buffer array

        // Check if a Mobile Terminated (MT) message was received
        // If no message is available, mtBufferSize = 0
        if (mtBufferSize > 0)
        {
          DEBUG_PRINT("Info: MT-SBD message received. Size: ");
          DEBUG_PRINT(mtBufferSize); DEBUG_PRINTLN(" bytes.");

          // Print contents of mtBuffer in hexadecimal
          char tempData[20];
          DEBUG_PRINTLN("Byte\tHex");
          for (int i = 0; i < sizeof(mtBuffer); ++i)
          {
            // Write incoming message buffer to union/structure
            mtMessage.bytes[i] = mtBuffer[i];
            sprintf(tempData, "%d\t0x%02X", i, mtBuffer[i]);
            DEBUG_PRINTLN(tempData);
          }

          // Print MT-SBD message as stored in union/structure
          printMtSbd();

          // Check if MT-SBD message data is valid and update global variables accordingly
          if ((mtMessage.alarmInterval      >= 300  && mtMessage.alarmInterval    <= 1209600) &&
              (mtMessage.transmitInterval   >= 1    && mtMessage.transmitInterval <= 24) &&
              (mtMessage.retransmitLimit    >= 0    && mtMessage.retransmitLimit  <= 24) &&
              (mtMessage.resetFlag          == 0    || mtMessage.resetFlag        == 255))
          {
            alarmInterval = mtMessage.alarmInterval;        // Update alarm interval
            transmitInterval = mtMessage.transmitInterval;  // Update transmit interval
            retransmitLimit = mtMessage.retransmitLimit;    // Update retransmit limit
            resetFlag = mtMessage.resetFlag;                // Update force reset flag
          }
          else
          {
            DEBUG_PRINT("Warning: Received values exceed accepted range! ");
          }
        }
      }
      else
      {
        DEBUG_PRINT("Warning: Transmission failed with error code ");
        DEBUG_PRINTLN(returnCode);
        setLedColourIridium(returnCode); // Set LED colour to return code
      }
    }

    // Write return error code to union
    moMessage.transmitStatus = returnCode;

    // Store message in transmit buffer if transmission or modem begin fails
    if (returnCode != ISBD_SUCCESS)
    {
      retransmitCounter++;
      failedTransmitCounter++;

      // Reset counter if reattempt limit is exceeded
      if (retransmitCounter > retransmitLimit)
      {
        retransmitCounter = 0;
        memset(transmitBuffer, 0x00, sizeof(transmitBuffer)); // Clear transmitBuffer array
      }
    }

    // Clear transmit buffer if program running for the first time
    if (firstTimeFlag)
    {
      retransmitCounter = 0;
      memset(transmitBuffer, 0x00, sizeof(transmitBuffer)); // Clear transmitBuffer array
    }

    // Put modem to sleep
    DEBUG_PRINTLN("Info: Putting modem to sleep...");
    returnCode = modem.sleep();
    if (returnCode != ISBD_SUCCESS)
    {
      DEBUG_PRINT("Warning: Sleep failed error "); DEBUG_PRINTLN(returnCode);
      setLedColourIridium(returnCode); // Set LED colour to appropriate return code
    }

    // Close the serial port connected to the RockBLOCK
    IRIDIUM_PORT.end();

    // Disable power to Iridium 9603
    //disableIridiumPower();

    // Reset transmit counter
    transmitCounter = 0;

    // Stop the loop timer
    timer.iridium = millis() - loopStartTime;

    // Write duration of last transmission to union
    moMessage.transmitDuration = timer.iridium / 1000;

    printSettings(); // Print current settings

    // Check if reset flag was transmitted
    if (resetFlag)
    {
      DEBUG_PRINTLN("Info: Forced system reset...");
      digitalWrite(LED_BUILTIN, HIGH); // Turn on LED
      while (true); // Wait for Watchdog Timer to reset system
    }
  }
}

// Non-blocking RockBLOCK callback function can be called during transmit or GPS signal acquisition
bool ISBDCallback()
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > 1000)
  {
    previousMillis = currentMillis;
    petDog(); // Reset the Watchdog Timer
    readBattery(); // Measure battery voltage during Iridium transmission
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Blink LED
  }
  return true;
}

#if DEBUG_IRIDIUM
// Callback to sniff the conversation with the Iridium modem
void ISBDConsoleCallback(IridiumSBD * device, char c)
{
  DEBUG_WRITE(c);
}

// Callback to to monitor Iridium modem library's run state
void ISBDDiagsCallback(IridiumSBD * device, char c)
{
  DEBUG_WRITE(c);
}
#endif

// Change LED colour to indicate return error code
void setLedColourIridium(byte err)
{
  if (err == 0) // ISBD_SUCCESS
    setLedColour(CRGB::Green);
  else if (err == 1) // ISBD_ALREADY_AWAKE
    setLedColour(CRGB::Yellow);
  else if (err == 2) // ISBD_SERIAL_FAILURE
    setLedColour(CRGB::Yellow);
  else if (err == 3) // ISBD_PROTOCOL_ERROR
    setLedColour(CRGB::Blue);
  else if (err == 4) // ISBD_CANCELLED
    setLedColour(CRGB::Yellow);
  else if (err == 5) // ISBD_NO_MODEM_DETECTED
    setLedColour(CRGB::Red);
  else if (err == 6) // ISBD_SBDIX_FATAL_ERROR
    setLedColour(CRGB::Yellow);
  else if (err == 7) // ISBD_SENDRECEIVE_TIMEOUT
    setLedColour(CRGB::Orange);
  else if (err == 8) // ISBD_RX_OVERFLOW
    setLedColour(CRGB::Yellow);
  else if (err == 9) // ISBD_REENTRANT
    setLedColour(CRGB::Yellow);
  else if (err == 10) // ISBD_IS_ASLEEP
    setLedColour(CRGB::Yellow);
  else if (err == 11) // ISBD_NO_SLEEP_PIN
    setLedColour(CRGB::Yellow);
  else if (err == 12) // ISBD_NO_NETWORK
    setLedColour(CRGB::Yellow);
  else if (err == 13) // ISBD_MSG_TOO_LONG
    setLedColour(CRGB::Yellow);
}

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
