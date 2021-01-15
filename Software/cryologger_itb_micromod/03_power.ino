// Read battery voltage from voltage divider
void readBattery()
{
  /*
    // Start loop timer
    unsigned long loopStartTime = millis();

    int reading = 0;
    byte samples = 30;

    for (byte i = 0; i < samples; ++i)
    {
    reading += analogRead(PIN_VBAT); // Read VIN across a 1/10 MΩ resistor divider
    delay(1);
    }

    voltage = (float)reading / samples * 3.3 * ((R2 + R1) / R2) / 4096.0; // Convert 1/10 VIN to VIN (12-bit resolution)

    // Write minimum battery voltage value to union
    if (moMessage.voltage == 0)
    {
    moMessage.voltage = voltage * 1000;
    }
    else if ((voltage * 1000) < moMessage.voltage)
    {
    moMessage.voltage = voltage * 1000;
    }

    //DEBUG_PRINT("voltage: "); DEBUG_PRINTLN(voltage);

    // Stop loop timer
    unsigned long loopEndTime = millis() - loopStartTime;
    //DEBUG_PRINT("readBattery() function execution: "); DEBUG_PRINT(loopEndTime); DEBUG_PRINTLN(" ms");
  */
}

// Enter deep sleep
void goToSleep()
{
  // Clear first-time flag after initial power-down
  if (firstTimeFlag) {
    firstTimeFlag = false;
  }
  
#if DEBUG
  Serial.flush();       // Wait for transmission of serial data to complete
  Serial.end();         // Disable Serial
#endif
  Wire.end();           // Disable I2C
  SPI.end();            // Disable SPI
  power_adc_disable();  // Disable power to ADC

  // Force peripherals off
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM0);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM1);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM2);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM3);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM4);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM5);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_ADC);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_UART0);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_UART1);

  // Disable all pads except G1, G2 and LED
  for (int x = 0; x < 50; x++)
  {
    if ((x != ap3_gpio_pin2pad(PIN_PWC_POWER)) &&
        (x != ap3_gpio_pin2pad(PIN_QWIIC_POWER)) &&
        (x != ap3_gpio_pin2pad(LED_BUILTIN)))
    {
      am_hal_gpio_pinconfig(x, g_AM_HAL_GPIO_DISABLE);
    }
  }

  qwiicPowerOff();      // Disable power to Qwiic connector
  peripheralPowerOff(); // Disable power to peripherals

  // Mark devices as offline
  online.bme280 = false;
  online.gnss = false;
  online.iridium = false;
  online.microSd = false;

  // Disable power to Flash, SRAM, and cache
  am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_CACHE); // Turn off CACHE
  am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_FLASH_512K); // Turn off everything but lower 512k
  am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_SRAM_64K_DTCM); // Turn off everything but lower 64k
  //am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_ALL); // Turn off all memory (doesn't recover)

  // Keep the 32kHz clock running for RTC
  am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
  am_hal_stimer_config(AM_HAL_STIMER_XTAL_32KHZ);

  // Enter deep sleep
  am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);

  /*
     Processor sleeps and awaits RTC or WDT ISR
  */

  // Wake up
  wakeUp();
}

// Wake from deep sleep
void wakeUp()
{
  // Enable power to SRAM, turn on entire Flash
  am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_MAX);

  // Return to using the main clock
  am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
  am_hal_stimer_config(AM_HAL_STIMER_HFRC_3MHZ);

  ap3_adc_setup();      // Enable power to ADC

  Wire.begin();         // Enable I2C
  SPI.begin();          // Enable SPI
#if DEBUG
  Serial.begin(115200); // Enable Serial
#endif

  // If alarm is triggered, enable power to system
  if (alarmFlag)
  {
    readRtc();            // Read RTC
    qwiicPowerOn();       // Enable power to Qwiic connector
    peripheralPowerOn();  // Enable power to peripherals
    configureSd();        // Configure microSD
    configureGnss();      // Configure SAM-M8Q receiver
    configureIridium();   // Configure Iridium 9603
    configureSensors();   // Configure attached sensors
  }
}

// Enable power to Qwiic connector
void qwiicPowerOn()
{
  digitalWrite(PIN_QWIIC_POWER, HIGH);
  // Non-blocking delay to allow Qwiic devices time to power up
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > qwiicPowerDelay)
  {
    previousMillis = currentMillis;
  }
}

// Disable power to Qwiic connector
void qwiicPowerOff()
{
  digitalWrite(PIN_QWIIC_POWER, LOW);
}

// Enable power to microSD and peripherals
void peripheralPowerOn()
{
  digitalWrite(PIN_PWC_POWER, HIGH);
}

// Disable power to microSD and peripherals
void peripheralPowerOff()
{
  digitalWrite(PIN_PWC_POWER, LOW);
}

// Non-blocking blink LED (https://forum.arduino.cc/index.php?topic=503368.0)
void blinkLed(byte ledFlashes, unsigned int ledDelay)
{
  byte i = 0;
  while (i < ledFlashes * 2)
  {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= ledDelay)
    {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      previousMillis = currentMillis;
      i++;
    }
  }
  // Ensure LED is off at end of blink cycle
  digitalWrite(LED_BUILTIN, LOW);
}
