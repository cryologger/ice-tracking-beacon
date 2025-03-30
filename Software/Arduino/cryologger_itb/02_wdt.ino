/*
  Watchdog Timer (WDT) Module

  This module configures the SAMD21 Watchdog Timer (WDT) to perform a system 
  reset if the main loop() is blocked for more than 8–16 seconds. It includes 
  functions to reset (pet) the Watchdog and an ISR to handle early warnings. 
*/

// ----------------------------------------------------------------------------
// Configure Watchdog Timer.
// Perform a system reset if loop() blocks for more than 8–16 seconds. 
// ----------------------------------------------------------------------------
void configureWdt() {
  /*
  // The following code block is used to configure the WDT only when using an 
  // external real-time clock (RTC) (i.e., when GCLK2 is not preconfigured by 
  // the internal RTC)
  // Configure Generic Clock Controller 2 (GCLK2) for use with WDT at 1.024kHz
  REG_GCLK_GENDIV = GCLK_GENDIV_ID(2) |  // Select Generic Clock Controller 2.
                    GCLK_GENDIV_DIV(4);  // Divide 32.768kHz clock source by divisor 32, where 2^(4 + 1): 32.768kHz/32=1.024kHz
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;  // Await synchronization of registers between clock domains

  REG_GCLK_GENCTRL = GCLK_GENCTRL_ID(2) |          // Select GCLK2
                     GCLK_GENCTRL_GENEN |          // Enable GCLK2
                     GCLK_GENCTRL_SRC_OSCULP32K |  // Set clock source to ultra low power oscillator (OSCULP32K)
                     GCLK_GENCTRL_DIVSEL |         // Set to divide by 2^(GCLK_GENDIV_DIV(4) + 1)
                     GCLK_GENCTRL_IDC;             // Set the duty cycle to 50/50 HIGH/LOW (REQUIRED?)
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;  // Await synchronization of registers between clock domains
  */

  // Feed GCLK2 to WDT (preconfigured by the RTC)
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_ID_WDT |    // Identify as the WDT clock
                     GCLK_CLKCTRL_CLKEN |     // Enable the generic clock by setting the Clock Enabler bit
                     GCLK_CLKCTRL_GEN_GCLK2;  // Select GCLK2
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;  // Await synchronization of registers between clock domains

  REG_WDT_EWCTRL = WDT_EWCTRL_EWOFFSET_8K;  // Set Early Warning Interrupt Time Offset to 8 seconds
  REG_WDT_INTENSET = WDT_INTENSET_EW;       // Enable Early Warning interrupt
  REG_WDT_CONFIG = WDT_CONFIG_PER_16K;      // Set WDT reset timeout to 16 seconds
  REG_WDT_CTRL = WDT_CTRL_ENABLE;           // Enable WDT in normal mode
  while (WDT->STATUS.bit.SYNCBUSY)
    ;  // Await synchronization of registers between clock domains

  // Configure and enable WDT interrupt
  NVIC_DisableIRQ(WDT_IRQn);
  NVIC_ClearPendingIRQ(WDT_IRQn);
  NVIC_SetPriority(WDT_IRQn, 0);  // Top priority
  NVIC_EnableIRQ(WDT_IRQn);
}

// ----------------------------------------------------------------------------
// Reset the Watchdog Timer.
// ----------------------------------------------------------------------------
void resetWdt() {
  //DEBUG_PRINT("[WDT] WDT interrupt = "); DEBUG_PRINTLN(wdtCounter);
  REG_WDT_CLEAR = WDT_CLEAR_CLEAR_KEY;  // Clear WDT and restart time-out period

  while (WDT->STATUS.bit.SYNCBUSY)
    ;  // Await synchronization of registers between clock domains

  wdtFlag = false;  
  wdtCounter = 0;   
}

// ----------------------------------------------------------------------------
// Watchdog Timer Interrupt Service Routine.
// ----------------------------------------------------------------------------
void WDT_Handler() {
  REG_WDT_INTFLAG = WDT_INTFLAG_EW;  // Clear Early Warning interrupt flag

  // Perform system reset after 10 WDT interrupts (should not occur)
  if (wdtCounter < 10) {
    REG_WDT_CLEAR = WDT_CLEAR_CLEAR_KEY;  // Clear WDT and restart time-out period
    while (WDT->STATUS.bit.SYNCBUSY)
      ;  // Await synchronization of registers between clock domains
  } else {
    //WDT->CTRL.bit.ENABLE = 0;         // Debugging only: Disable WDT
    //digitalWrite(LED_BUILTIN, HIGH);  // Debugging only: Turn on LED to indicate WDT trigger
    while (true)
      ;  // Force WDT to reset the system.
  }
  wdtFlag = true;  // Set the WDT flag
  wdtCounter++;    // Increment WDT interrupt counter
}
