void printLine() {
  for (byte i = 0; i < 79; i++) {
    Serial.print("-");
  }
  Serial.println();
}

void printTab(byte _times) {
  for (byte i = 0; i < _times; i++) {
    Serial.print("\t");
  }
}

// Print contents of union/structure
void printUnion() {
  printLine();
  Serial.println(F("Union/structure"));
  printLine();
  Serial.print(F("unixtime:\t\t")); Serial.println(message.unixtime);
  Serial.print(F("temperature:\t\t")); Serial.println(message.temperature);
  Serial.print(F("humidity:\t\t")); Serial.println(message.humidity);
  Serial.print(F("pressure:\t\t")); Serial.println(message.pressure);
  //Serial.print(F("pitch:\t\t\t")); Serial.println(message.pitch);
  //Serial.print(F("roll:\t\t\t")); Serial.println(message.roll);
  //Serial.print(F("heading:\t\t")); Serial.println(message.heading);
  Serial.print(F("latitude:\t\t")); Serial.println(message.latitude);
  Serial.print(F("longitude:\t\t")); Serial.println(message.longitude);
  Serial.print(F("satellites:\t\t")); Serial.println(message.satellites);
  Serial.print(F("pdop:\t\t\t")); Serial.println(message.pdop);
  Serial.print(F("fix:\t\t\t")); Serial.println(message.fix);
  //Serial.print(F("voltage:\t\t")); Serial.println(message.voltage);
  Serial.print(F("transmitDuration:\t")); Serial.println(message.transmitDuration);
  Serial.print(F("messageCounter:\t\t")); Serial.println(message.messageCounter);
  printLine();
}

// Print contents of union/structure in binary and hexadecimal
void printUnionBinary() {
  Serial.println(F("Union/structure "));
  printLine();
  Serial.println(F("Byte\tHex\tBinary"));
  for (int i = 0; i < sizeof(message); ++i) {
    Serial.print(i);
    Serial.print("\t");
    Serial.print(message.bytes[i], HEX);
    Serial.print("\t");
    Serial.println(message.bytes[i], BIN);
  }
  printLine();
}

// Print contents of transmit buffer in binary and hexadecimal
void printTransmitBuffer() {
  Serial.println(F("Transmit buffer"));
  printLine();
  Serial.println(F("Byte\tHex\tBinary"));
  for (int i = 0; i < 340; i++) {
    Serial.print(i);
    Serial.print(F("\t"));
    Serial.print(transmitBuffer[i], HEX);
    Serial.print("\t");
    Serial.println(transmitBuffer[i], BIN);
  }
}
