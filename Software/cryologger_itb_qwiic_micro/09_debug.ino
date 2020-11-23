void printLine() {
  for (byte i = 0; i < 79; i++) {
    SERIAL_PORT.print("-");
  }
  SERIAL_PORT.println();
}

void printTab(byte _times) {
  for (byte i = 0; i < _times; i++) {
    SERIAL_PORT.print("\t");
  }
}

// Print contents of union/structure
void printUnion() {
  printLine();
  SERIAL_PORT.println(F("Union/structure"));
  printLine();
  SERIAL_PORT.print(F("unixtime:")); printTab(2); SERIAL_PORT.println(message.unixtime);
  SERIAL_PORT.print(F("temperature:")); printTab(2); SERIAL_PORT.println(message.temperature);
  SERIAL_PORT.print(F("humidity:")); printTab(2); SERIAL_PORT.println(message.humidity);
  SERIAL_PORT.print(F("pressure:")); printTab(2); SERIAL_PORT.println(message.pressure);
  //SERIAL_PORT.print(F("pitch:")); printTab(3); SERIAL_PORT.println(message.pitch);
  //SERIAL_PORT.print(F("roll:")); printTab(3); SERIAL_PORT.println(message.roll);
  //SERIAL_PORT.print(F("heading:")); printTab(2); SERIAL_PORT.println(message.heading);
  SERIAL_PORT.print(F("latitude:")); printTab(2); SERIAL_PORT.println(message.latitude);
  SERIAL_PORT.print(F("longitude:")); printTab(2); SERIAL_PORT.println(message.longitude);
  SERIAL_PORT.print(F("satellites:")); printTab(2); SERIAL_PORT.println(message.satellites);
  SERIAL_PORT.print(F("pdop:")); printTab(3); SERIAL_PORT.println(message.pdop);
  SERIAL_PORT.print(F("voltage:")); printTab(2); SERIAL_PORT.println(message.voltage);
  SERIAL_PORT.print(F("transmitDuration:")); printTab(1); SERIAL_PORT.println(message.transmitDuration);
  SERIAL_PORT.print(F("messageCounter:")); printTab(2); SERIAL_PORT.println(message.messageCounter);
  printLine();
}

// Print contents of union/structure in binary and hexadecimal
void printUnionBinary() {
  SERIAL_PORT.println(F("Union/structure "));
  printLine();
  SERIAL_PORT.println(F("Byte\tHex\tBinary"));
  for (int i = 0; i < sizeof(message); ++i) {
    SERIAL_PORT.print(i);
    printTab(1); 
    SERIAL_PORT.print(message.bytes[i], HEX);
    printTab(1); 
    SERIAL_PORT.println(message.bytes[i], BIN);
  }
  printLine();
}

// Print contents of transmit buffer in binary and hexadecimal
void printTransmitBuffer() {
  SERIAL_PORT.println(F("Transmit buffer"));
  printLine();
  SERIAL_PORT.println(F("Byte\tHex\tBinary"));
  for (int i = 0; i < 340; i++) {
    SERIAL_PORT.print(i);
    printTab(1); 
    SERIAL_PORT.print(transmitBuffer[i], HEX);
    printTab(1); 
    SERIAL_PORT.println(transmitBuffer[i], BIN);
  }
}
