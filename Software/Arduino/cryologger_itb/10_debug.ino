/*
  Debug Module

  This module provides debugging utilities to print user-defined system
  settings, function execution timers, Iridium SBD message data, and available
  memory. 
*/

// ----------------------------------------------------------------------------
// Prints a horizontal separator line (80 dashes).
// ----------------------------------------------------------------------------
void printLine() {
  for (byte i = 0; i < 80; i++) {
    DEBUG_PRINT("-");
  }
  DEBUG_PRINTLN();
}

// ----------------------------------------------------------------------------
// Prints tab spacing. Each tab is displayed `_times` times.
// ----------------------------------------------------------------------------
void printTab(byte _times) {
  for (byte i = 0; i < _times; i++) {
    DEBUG_PRINT("\t");
  }
}

// ----------------------------------------------------------------------------
// Prints user-defined beacon settings, including any global variables
// that define transmit intervals, counters, and system voltage.
// ----------------------------------------------------------------------------
void printSettings() {
  printLine();
  DEBUG_PRINTLN("Current Settings");
  printLine();

  DEBUG_PRINT("retransmitCounter: ");
  printTab(1);
  DEBUG_PRINTLN(transmitInterval);

  DEBUG_PRINT("retransmitCounter: ");
  printTab(1);
  DEBUG_PRINTLN(retransmitCounter);

  DEBUG_PRINT("retransmitLimit: ");
  printTab(1);
  DEBUG_PRINTLN(retransmitLimit);

  DEBUG_PRINT("resetFlag: ");
  printTab(2);
  DEBUG_PRINTLN(resetFlag);

  DEBUG_PRINT("voltage: ");
  printTab(2);
  DEBUG_PRINTLN(voltage);

  printLine();
}

// ----------------------------------------------------------------------------
// Prints function execution timers stored in the global 'timer' structure.
// Displays how long various sensor reads and operations took.
// ----------------------------------------------------------------------------
void printTimers() {
  //printLine();
  DEBUG_PRINTLN("Function Execution Timers");
  printLine();

  DEBUG_PRINT("readBattery: ");
  printTab(1);
  DEBUG_PRINTLN(timer.readBattery);

  DEBUG_PRINT("readRtc: ");
  printTab(1);
  DEBUG_PRINTLN(timer.readRtc);

  DEBUG_PRINT("readBme280: ");
  printTab(1);
  DEBUG_PRINTLN(timer.readBme280);

  DEBUG_PRINT("readLsm6dsox: ");
  printTab(1);
  DEBUG_PRINTLN(timer.readLsm6dsox);

  DEBUG_PRINT("readGnss: ");
  printTab(1);
  DEBUG_PRINTLN(timer.readGnss);

  DEBUG_PRINT("transmitData: ");
  printTab(1);
  DEBUG_PRINTLN(timer.iridium);

  DEBUG_PRINT("freeRam(): ");
  printTab(1);
  DEBUG_PRINTLN(freeRam());

  printLine();
}

// ----------------------------------------------------------------------------
// Clears all execution timers.
// Resets the timer structure to zero for performance diagnostics.
// ----------------------------------------------------------------------------
void clearTimers() {
  memset(&timer, 0x00, sizeof(timer));
}

// ----------------------------------------------------------------------------
// Prints Mobile Originated (MO) SBD message data stored in 'moSbdMessage'.
// Shows each field in a human-readable format.
// ----------------------------------------------------------------------------
void printMoSbd() {
  printLine();
  DEBUG_PRINTLN("MO-SBD Message Data");
  printLine();

  DEBUG_PRINT("unixtime:");
  printTab(2);
  DEBUG_PRINTLN(moSbdMessage.unixtime);

  DEBUG_PRINT("temperature:");
  printTab(2);
  DEBUG_PRINTLN(moSbdMessage.temperatureInt);

  DEBUG_PRINT("humidity:");
  printTab(2);
  DEBUG_PRINTLN(moSbdMessage.humidityInt);

  DEBUG_PRINT("pressure:");
  printTab(2);
  DEBUG_PRINTLN(moSbdMessage.pressureInt);

  DEBUG_PRINT("pitch:");
  printTab(3);
  DEBUG_PRINTLN(moSbdMessage.pitch);

  DEBUG_PRINT("roll:");
  printTab(3);
  DEBUG_PRINTLN(moSbdMessage.roll);

  DEBUG_PRINT("heading:");
  printTab(2);
  DEBUG_PRINTLN(moSbdMessage.heading);

  DEBUG_PRINT("latitude:");
  printTab(2);
  DEBUG_PRINTLN(moSbdMessage.latitude);

  DEBUG_PRINT("longitude:");
  printTab(2);
  DEBUG_PRINTLN(moSbdMessage.longitude);

  DEBUG_PRINT("satellites:");
  printTab(2);
  DEBUG_PRINTLN(moSbdMessage.satellites);

  DEBUG_PRINT("hdop:");
  printTab(3);
  DEBUG_PRINTLN(moSbdMessage.hdop);

  DEBUG_PRINT("voltage:");
  printTab(2);
  DEBUG_PRINTLN(moSbdMessage.voltage);

  DEBUG_PRINT("transmitDuration:");
  printTab(1);
  DEBUG_PRINTLN(moSbdMessage.transmitDuration);

  DEBUG_PRINT("transmitStatus:");
  printTab(2);
  DEBUG_PRINTLN(moSbdMessage.transmitStatus);

  DEBUG_PRINT("iterationCounter:");
  printTab(1);
  DEBUG_PRINTLN(moSbdMessage.iterationCounter);

  printLine();
}

// ----------------------------------------------------------------------------
// Prints Mobile Terminated (MT) SBD message data stored in 'mtSbdMessage'.
// Shows each field in a human-readable format.
// ----------------------------------------------------------------------------
void printMtSbd() {
  printLine();
  DEBUG_PRINTLN("MT-SBD Message Data");
  printLine();

  DEBUG_PRINT("transmitInterval:");
  printTab(1);
  DEBUG_PRINTLN(mtSbdMessage.transmitInterval);

  DEBUG_PRINT("retransmitLimit:");
  printTab(1);
  DEBUG_PRINTLN(mtSbdMessage.retransmitLimit);

  DEBUG_PRINT("resetFlag:");
  printTab(2);
  DEBUG_PRINTLN(mtSbdMessage.resetFlag);

  printLine();
}

// ----------------------------------------------------------------------------
// Prints the contents of the MO-SBD union/structure in a hex dump format.
// ----------------------------------------------------------------------------
void printMoSbdHex() {
  DEBUG_PRINTLN("MO-SBD Union/structure ");
  printLine();

  char tempData[16];
  DEBUG_PRINTLN("Byte\tHex");

  for (int i = 0; i < (int)sizeof(moSbdMessage); ++i) {
    sprintf(tempData, "%d\t0x%02X", i, moSbdMessage.bytes[i]);
    DEBUG_PRINTLN(tempData);
  }

  printLine();
}

// ----------------------------------------------------------------------------
// Prints the MO-SBD transmit buffer (moSbdBuffer) in a hex dump format.
// ----------------------------------------------------------------------------
void printMoSbdBuffer() {
  printLine();
  DEBUG_PRINTLN("MO-SBD Transmit buffer");
  printLine();

  char tempData[16];
  DEBUG_PRINTLN("Byte\tHex");

  for (int i = 0; i < (int)moSbdBufferSize; ++i) {
    sprintf(tempData, "%d\t0x%02X", i, moSbdBuffer[i]);
    DEBUG_PRINTLN(tempData);
  }
}

// ----------------------------------------------------------------------------
// Prints the MT-SBD transmit buffer (mtSbdBuffer) in a hex dump format.
// ----------------------------------------------------------------------------
void printMtSbdBuffer() {
  printLine();
  DEBUG_PRINTLN("MT-SBD Transmit buffer");
  printLine();

  char tempData[16];
  DEBUG_PRINTLN("Byte\tHex");

  for (int i = 0; i < (int)mtSbdBufferSize; ++i) {
    sprintf(tempData, "%d\t0x%02X", i, mtSbdBuffer[i]);
    DEBUG_PRINTLN(tempData);
  }
}

// ----------------------------------------------------------------------------
// Computes and returns the amount of free RAM, in bytes.
// ----------------------------------------------------------------------------
extern "C" char *sbrk(int i);
int freeRam() {
  char stack_dummy = 0;
  return &stack_dummy - sbrk(0);
}
