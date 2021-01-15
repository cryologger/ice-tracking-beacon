/*
  Author: Adam Garbo
  Created: November 24th, 2020

  This example demonstrates how to place the MicroMod Data Logging Carrier Board and Artemis Processor into deep sleep.

  The RTC will wake the system every minute, print the next rolling alarm, and flash the LED. 

  Current consumption is measured across the BATT jumpers with a 3.7V LiPo battery. Absolute minimum current draw acheived 
  with the onboard voltage divider disabled was ~280 uA.

  Note: 
  - When disabling all pads, it is important not to include G1/G2, which control the state of the two AP2112 LDO regulators

*/

#include <RTC.h>
#include <SPI.h>
#include <Wire.h>

const byte peripheralPowerControl = 33;
const byte qwiicPowerControl = 34;
const byte sdChipSelect = 41;

APM3_RTC myRTC;

volatile bool alarmFlag       = false;
bool ledState                 = LOW;    // LED toggle flag for blink() function
byte alarmSeconds             = 30;
byte alarmMinutes             = 0;
byte alarmHours               = 0;
unsigned long previousMillis  = 0;

void setup()
{
  pinMode(qwiicPowerControl, OUTPUT);
  pinMode(peripheralPowerControl, OUTPUT);

  qwiicPowerOn();       // Enable Qwiic power
  peripheralPowerOn();  // Enable power to peripherials

  Wire.begin(); // Initialize I2C
  SPI.begin();  // Initialize SPI

  Serial.begin(115200);
  Serial.println("SparkFun MicroMod Data Logging Carrier Board Low-power Alarm Example");

  // Manually set RTC date and time
  myRTC.setTime(12, 59, 50, 0, 24, 11, 20); // (hh, mm, ss, hund, dd, mm, yy)

  // Set the RTC's alarm
  myRTC.setAlarm(13, 0, 0, 0, 24, 11); // (hh, mm, ss, hund, dd, mm)

  // Set the RTC alarm mode
  myRTC.setAlarmMode(6); // Set the RTC alarm to match on minutes rollover
  myRTC.attachInterrupt(); // Attach RTC alarm interrupt
}

void loop()
{
  // Check if alarm flag was set
  if (alarmFlag == true)
  {
    // Print date and time of RTC alarm trigger
    Serial.print("Alarm triggered: "); printDateTime();

    // Clear alarm flag
    alarmFlag = false;

    // Set the RTC's rolling alarm
    myRTC.setAlarm((myRTC.hour + alarmHours) % 24,
                   (myRTC.minute + alarmMinutes) % 60,
                   (myRTC.seconds + alarmSeconds) % 60,
                   0, myRTC.dayOfMonth, myRTC.month);
    myRTC.setAlarmMode(5);

    // Print next RTC alarm date and time
    Serial.print("Next rolling alarm: "); printAlarm();
  }

  blinkLed(1, 2000);

  // Enter deep sleep and await RTC alarm interrupt
  goToSleep();
}

// Print the RTC's current date and time
void printDateTime()
{
  myRTC.getTime();
  char dateTimeBuffer[25];
  sprintf(dateTimeBuffer, "20%02d-%02d-%02d %02d:%02d:%02d.%03d",
          myRTC.year, myRTC.month, myRTC.dayOfMonth,
          myRTC.hour, myRTC.minute, myRTC.seconds, myRTC.hundredths);
  Serial.println(dateTimeBuffer);
}

// Print the RTC's alarm
void printAlarm()
{
  myRTC.getAlarm();
  char alarmBuffer[25];
  sprintf(alarmBuffer, "2020-%02d-%02d %02d:%02d:%02d.%03d",
          myRTC.alarmMonth, myRTC.alarmDayOfMonth,
          myRTC.alarmHour, myRTC.alarmMinute,
          myRTC.alarmSeconds, myRTC.alarmHundredths);
  Serial.println(alarmBuffer);
}

// Power down gracefully
void goToSleep()
{
  Wire.end();           // Power down I2C
  SPI.end();            // Power down SPI
  power_adc_disable();  // Power down ADC
  Serial.end();         // Power down UART

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

  // Disable pads
  for (int x = 0; x < 50; x++)
  {
    if ((x != ap3_gpio_pin2pad(peripheralPowerControl)) &&
        (x != ap3_gpio_pin2pad(qwiicPowerControl)))
    {
      am_hal_gpio_pinconfig(x, g_AM_HAL_GPIO_DISABLE);
    }
  }

  qwiicPowerOff();
  peripheralPowerOff();

  // Power down Flash, SRAM, cache
  am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_CACHE); // Turn off CACHE
  am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_FLASH_512K); // Turn off everything but lower 512k
  am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_SRAM_64K_DTCM); // Turn off everything but lower 64k
  //am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_ALL); //Turn off all memory (doesn't recover)

  // Keep the 32kHz clock running for RTC
  am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
  am_hal_stimer_config(AM_HAL_STIMER_XTAL_32KHZ);

  am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP); // Sleep forever

  // And we're back!
  wakeUp();
}

// Power up gracefully
void wakeUp()
{
  // Power up SRAM, turn on entire Flash
  am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_MAX);

  // Go back to using the main clock
  am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
  am_hal_stimer_config(AM_HAL_STIMER_HFRC_3MHZ);

  ap3_adc_setup();  // Power up ADC
  Wire.begin();     // Power up I2C
  SPI.begin();      // Power up SPI

  // Enable Serial
  Serial.begin(115200);

  //qwiicPowerOn();
  //peripheralPowerOn();

}

// Interrupt handler for the RTC
extern "C" void am_rtc_isr(void)
{
  // Clear the RTC alarm interrupt.
  am_hal_rtc_int_clear(AM_HAL_RTC_INT_ALM);

  // Set alarm flag
  alarmFlag = true;
}

//Qwiic connector power is controlled by LDO voltage regulator
void qwiicPowerOn()
{
  digitalWrite(qwiicPowerControl, HIGH);
}
void qwiicPowerOff()
{
  digitalWrite(qwiicPowerControl, LOW);
}

//3V3 peripheral power is controlled by LDO voltage regulator
void peripheralPowerOn()
{
  digitalWrite(peripheralPowerControl, HIGH);
}
void peripheralPowerOff()
{
  digitalWrite(peripheralPowerControl, LOW);
}

void ledOn()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Turn the STAT LED on
}

void ledOff()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); // Turn the STAT LED off
}

// Blink LED (non-blocking)
void blinkLed(byte flashes, unsigned int interval) {

  pinMode(LED_BUILTIN, OUTPUT);
  uint8_t i = 0;

  while (i <= flashes * 2) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      if (ledState == LOW) {
        ledState = HIGH;
      }
      else {
        ledState = LOW;
      }
      digitalWrite(LED_BUILTIN, ledState);
      i++;
    }
  }
}
