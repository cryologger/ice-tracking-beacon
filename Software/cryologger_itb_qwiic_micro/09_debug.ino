void printLine()
{
  for (byte i = 0; i < 80; i++)
  {
    DEBUG_PRINT("-");
  }
  DEBUG_PRINTLN();
}

void printTab(byte _times)
{
  for (byte i = 0; i < _times; i++)
  {
    DEBUG_PRINT("\t");
  }
}

// Print union/structure
void printUnion() {
  printLine();
  DEBUG_PRINTLN("Union/structure");
  printLine();
  DEBUG_PRINT("unixtime:");         printTab(2);  DEBUG_PRINTLN(moMessage.unixtime);
  DEBUG_PRINT("temperature:");      printTab(2);  DEBUG_PRINTLN(moMessage.temperature);
  DEBUG_PRINT("humidity:");         printTab(2);  DEBUG_PRINTLN(moMessage.humidity);
  DEBUG_PRINT("pressure:");         printTab(2);  DEBUG_PRINTLN(moMessage.pressure);
  //DEBUG_PRINT("pitch:");            printTab(3);  DEBUG_PRINTLN(moMessage.pitch);
  //DEBUG_PRINT("roll:");             printTab(3);  DEBUG_PRINTLN(moMessage.roll);
  //DEBUG_PRINT("heading:");          printTab(2);  DEBUG_PRINTLN(moMessage.heading);
  DEBUG_PRINT("latitude:");         printTab(2);  DEBUG_PRINTLN(moMessage.latitude);
  DEBUG_PRINT("longitude:");        printTab(2);  DEBUG_PRINTLN(moMessage.longitude);
  DEBUG_PRINT("satellites:");       printTab(2);  DEBUG_PRINTLN(moMessage.satellites);
  DEBUG_PRINT("pdop:");             printTab(3);  DEBUG_PRINTLN(moMessage.pdop);
  DEBUG_PRINT("rtcDrift:");         printTab(2);  DEBUG_PRINTLN(moMessage.rtcDrift);
  DEBUG_PRINT("voltage:");          printTab(2);  DEBUG_PRINTLN(moMessage.voltage);
  DEBUG_PRINT("transmitDuration:"); printTab(1);  DEBUG_PRINTLN(moMessage.transmitDuration);
  DEBUG_PRINT("messageCounter:");   printTab(2);  DEBUG_PRINTLN(moMessage.messageCounter);
  printLine();
}

// Print contents of union/structure
void printUnionHex() 
{
  DEBUG_PRINTLN("Union/structure ");
  printLine();
  char tempData[340];
  DEBUG_PRINTLN("Byte\tHex");
  for (int i = 0; i < sizeof(moMessage); ++i) 
  {
    sprintf(tempData, "%d\t0x%02X", i, moMessage.bytes[i]);
    DEBUG_PRINTLN(tempData);
  }
  printLine();
}

// Print contents of transmit buffer
void printTransmitBuffer() 
{
  DEBUG_PRINTLN("Transmit buffer");
  printLine();
  char tempData[sizeof(transmitBuffer)];
  DEBUG_PRINTLN("Byte\tHex");
  for (int i = 0; i < sizeof(transmitBuffer); ++i) 
  {
    sprintf(tempData, "%d\t0x%02X", i, transmitBuffer[i]);
    DEBUG_PRINTLN(tempData);
  }
}
