// Enter deep sleep
void goToSleep() {

  Wire.end();           // Disable I2C
  SPI.end();            // Disable SPI
  Serial.end();         // Disable Serial
  power_adc_disable();  // Disable power to ADC

  // Force the peripherals off
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM0);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM1);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM2);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM3);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM4);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM5);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_ADC);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_UART0);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_UART1);

  // Disable all pads
  for (int x = 0; x < 50; x++)
  {
    if ((x != ap3_gpio_pin2pad(PIN_PWC_POWER)) &&
        (x != ap3_gpio_pin2pad(PIN_QWIIC_POWER)))
    {
      am_hal_gpio_pinconfig(x, g_AM_HAL_GPIO_DISABLE);
    }
  }

  qwiicPowerOff();      // Disable power to Qwiic connector
  peripheralPowerOff(); // Disable power to peripherals

  // Disable power to Flash, SRAM, and cache
  am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_CACHE); // Turn off CACHE
  am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_FLASH_512K); // Turn off everything but lower 512k
  am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_SRAM_64K_DTCM); // Turn off everything but lower 64k
  //am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_ALL); //Turn off all memory (doesn't recover)

  // Keep the 32kHz clock running for RTC
  am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
  am_hal_stimer_config(AM_HAL_STIMER_XTAL_32KHZ);

  // Enter deep sleep
  am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);

  // Wake
  wakeUp();

}

// Wake from sleep
void wakeUp() {

  // Enable power to SRAM, turn on entire Flash
  am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_MAX);

  // Return to using the main clock
  am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
  am_hal_stimer_config(AM_HAL_STIMER_HFRC_3MHZ);

  ap3_adc_setup();      // Enable power to ADC
  Wire.begin();         // Enable I2C
  SPI.begin();          // Enable SPI
  Serial.begin(115200); // Enable Serial
  qwiicPowerOn();       // Enable power to Qwiic connector
  peripheralPowerOn();  // Enable power to peripherals

  configureSd();      // Configure microSD
  configureGnss();    // Configure u-blox SAM-M8Q receiver
  configureIridium(); // Configure Qwiic Iridium 9603N
}

// Enable power to Qwiic connector
void qwiicPowerOn() {
  digitalWrite(PIN_QWIIC_POWER, HIGH);
}

// Disable power to Qwiic connector
void qwiicPowerOff() {
  digitalWrite(PIN_QWIIC_POWER, LOW);
}

// Enable power to peripherals
void peripheralPowerOn() {
  digitalWrite(PIN_PWC_POWER, HIGH);
}

// Disable power to peripherals
void peripheralPowerOff() {
  digitalWrite(PIN_PWC_POWER, LOW);
}
