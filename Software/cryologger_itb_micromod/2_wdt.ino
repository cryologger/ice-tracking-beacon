void configureWdt() {
  /*
    Simplified WDT Clock Divider Selections
    WDT_OFF     = Low Power Mode. This setting disables the watch dog timer
    WDT_128HZ   = 28 Hz LFRC clock
    WDT_16HZ    = 16 Hz LFRC clock
    WDT_1HZ     = 1 Hz LFRC clock
    WDT_1_16HZ  = 1/16th Hz LFRC clock
  */
  // Configure the Watchdog Timer
  wdt.configure(WDT_16HZ, 160, 240); // 16 Hz clock, 10-second interrupt period, 15-second reset period
  //wdt.configure(WDT_1HZ, 32, 64); // 1 Hz clock, 32-second interrupt period, 64-second reset period

  // Start the Watchdog
  wdt.start();
}

void petDog() {
  Serial.print("Watchdog interrupt: "); Serial.println(watchdogCounter);
  watchdogFlag = false; // Clear watchdog flag
  watchdogCounter = 0; // Reset watchdog interrupt counter
}
