// Configure SparkFun Qwiic Iridium 9603N
void configureIridium() {
  
  if (modem.isConnected()) {
    modem.adjustATTimeout(15);            // Set AT timeout (Default = 20 seconds)
    modem.adjustSendReceiveTimeout(150);  // Set send/receive timeout (Default = 300 seconds)
    modem.enable841lowPower(true);        // Enable ATtiny841 low-power mode
  }
  else {
    Serial.println(F("Warning: Qwiic Iridium 9603N not detected! Please check wiring."));
    //while (1);
  }
}

// Transmit data using SparkFun Qwiic Iridium 9603N
void transmitData() {

  // Loop timer
  unsigned long loopStartTime = millis();
  int err;

  // Enable the supercapacitor charger
  Serial.println(F("Enabling the supercapacitor charger..."));
  modem.enableSuperCapCharger(true);

  // Wait for supercapacitor charger PGOOD signal to go HIGH for up to 2 minutes
  while ((!modem.checkSuperCapCharger()) && millis() - loopStartTime < 2UL * 60UL * 1000UL) {
    //Serial.println(F("Waiting for supercapacitors to charge..."));
    blinkLed(1,1000);
    //petDog();
  }
  Serial.println(F("Supercapacitors charged!"));

  // Enable power for the Qwiic 9603N
  Serial.println(F("Enabling 9603N power..."));
  modem.enable9603Npower(true);

  // Begin satellite modem operation
  Serial.println(F("Starting modem..."));
  err = modem.begin();
  if (err == ISBD_SUCCESS) {
    uint8_t inBuffer[240];  // Buffer to store incoming transmission (240 byte limit)
    size_t inBufferSize = sizeof(inBuffer);
    memset(inBuffer, 0x00, sizeof(inBuffer)); // Clear inBuffer array

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

    /*
      // Transmit and receieve data in binary format
      err = modem.sendReceiveSBDBinary(transmitBuffer, (sizeof(message) * (transmitCounter + (retransmitCounter * transmitInterval))), inBuffer, inBufferSize);


      // Check if transmission was successful
      if (err == ISBD_SUCCESS) {
      retransmitCounter = 0;
      memset(transmitBuffer, 0x00, sizeof(transmitBuffer)); // Clear transmit buffer array
      Serial.println(F("Success!"));
      }
      else {
      Serial.print(F("Transmission failed: error "));
      Serial.println(err);
      }
    */
  }
  else {
    Serial.print(F("Begin failed: error "));
    Serial.println(err);
    if (err == ISBD_NO_MODEM_DETECTED) {
      Serial.println(F("No modem detected: check wiring."));
    }
    return;
  }

  // Store message in transmit buffer if transmission or modem begin fails
  if (err != ISBD_SUCCESS) {
    retransmitCounter++;
    // Reset counter if reattempt limit is exceeded
    if (retransmitCounter >= maxRetransmitCounter) {
      retransmitCounter = 0;
      memset(transmitBuffer, 0x00, sizeof(transmitBuffer));   // Clear transmitBuffer array
    }
  }

  // Power down the modem
  Serial.println(F("Putting the Qwiic 9603N to sleep."));
  err = modem.sleep();
  if (err != ISBD_SUCCESS) {
    Serial.print(F("Sleep failed: error "));
    Serial.println(err);
  }

  // Disable 9603N power
  Serial.println(F("Disabling 9603N power..."));
  modem.enable9603Npower(false);

  // Disable the supercapacitor charger
  Serial.println(F("Disabling the supercapacitor charger..."));
  modem.enableSuperCapCharger(false);

  // Enable the ATtiny841 low power mode
  Serial.println(F("Enabling ATtiny841 low power mode"));
  modem.enable841lowPower(true);


  unsigned long transmitDuration = millis() - loopStartTime;
  message.transmitDuration = transmitDuration / 1000;
  unsigned long loopEndTime = millis() - loopStartTime;

  Serial.print(F("transmitData() function execution: ")); Serial.print(loopEndTime); Serial.println(F(" ms"));
  Serial.print(F("transmitDuration: ")); Serial.println(transmitDuration / 1000);
  Serial.print(F("retransmitCounter: ")); Serial.println(retransmitCounter);

}

// RockBLOCK callback function that can be repeatedly called during data transmission or GNSS signal acquisitionn
bool ISBDCallback() {
#if DEBUG
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, (millis() / 1000) % 2 == 1 ? HIGH : LOW);
#endif

  unsigned int currentMillis = millis();
  if (currentMillis - previousMillis >= 2000)
  {
    previousMillis = currentMillis;
    //readBattery(); // Measure battery voltage
    //petDog();  // Pet the dog
  }
  return true;
}
#if DIAGNOSTICS
void ISBDConsoleCallback(IridiumSBD *device, char c)
{
  Serial.write(c);
}

void ISBDDiagsCallback(IridiumSBD *device, char c)
{
  Serial.write(c);
}
#endif

// Write union data to transmit buffer in preparation of data transmission
void writeBuffer() {
  messageCounter++;                         // Increment message counter
  message.messageCounter = messageCounter;  // Write message counter data to union
  transmitCounter++;                        // Increment data transmission counter

  // Concatenate current message with existing message(s) stored in transmit buffer
  memcpy(transmitBuffer + (sizeof(message) * (transmitCounter + (retransmitCounter * transmitInterval) - 1)), message.bytes, sizeof(message)); // Copy message to transmit buffer

#if DEBUG
  printUnion();
  printUnionBinary(); // Print union/structure in hex/binary
  printTransmitBuffer();  // Print transmit buffer in hex/binary
#endif
}
