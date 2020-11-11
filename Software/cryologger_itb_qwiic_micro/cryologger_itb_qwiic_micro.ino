/*
    Title:    Cryologger Ice Tracking Beacon (ITB) - Version 3
    Date:     November 11, 2020
    Author:   Adam Garbo

    Components:
    - SparkFun Qwiic Micro - SAMD21 Development Board
    - SparkFun Atmospheric Sensor Breakout - BME280 (Qwiic)
    - SparkFun GPS Breakout - SAM-M8Q (Qwiic)
    - SparkFun Qwiic Iridium 9603N
    - Maxtena M1621HCT-P-SMA Iridium antenna

    - SparkFun Buck-Boost Converter

    Comments:
    - Code is currently under development for the next iteration of the Cryologger iceberg
    tracking beacon to be deployed during the 2021 Amundsen Expedition.
    - The MicroMod system has potential, but it's really a pain in the ass.
    - With the addition of the Qwiic Power Switch and the Buck-Boost Converter feeding the
    3.3V bus directly, we're looking at 110-141 uA in sleep mode (total) at 7.4-12V.
    - Artemis MicroMod is currently sitting at 300 uA.
*/

// Libraries
#include <Wire.h>                           // https://www.arduino.cc/en/Reference/Wire
#include <SPIMemory.h>                      // https://github.com/Marzogh/SPIMemory
#include <SparkFun_Ublox_Arduino_Library.h> // https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library
#include <SparkFun_Qwiic_Power_Switch_Arduino_Library.h>
#include <SparkFunBME280.h>                 // https://github.com/sparkfun/SparkFun_BME280_Arduino_Library
#include <RTCZero.h>                        // https://github.com/arduino-libraries/RTCZero
#include <IridiumSBD.h>                     // https://github.com/sparkfun/SparkFun_IridiumSBD_I2C_Arduino_Library
#include <ArduinoLowPower.h>                // https://github.com/arduino-libraries/ArduinoLowPower

// Defined constants
#define Serial        SerialUSB   // Required by SparkFun Qwiic Micro 

#define DEBUG         true        // Output debugging messages to Serial Monitor
#define DIAGNOSTICS   true        // Output Iridium diagnostic messages to Serial Monitor

// Object instantiations
BME280        bme280;             // I2C Address: 0x77
IridiumSBD    modem(Wire);        // I2C Address: 0x63
QWIIC_POWER   mySwitch;           // I2C Address:
RTCZero       rtc;
SFE_UBLOX_GPS gps;                // I2C Address: 0x42
//SPIFlash      flash(21, &SPI1);

// User defined global variable declarations
uint32_t      alarmInterval         = 60;    // RTC sleep duration in seconds (Default: 3600 seconds)
uint8_t       transmitInterval      = 4;      // Number of messages in each Iridium transmission (Limit: 340 bytes)
uint8_t       maxRetransmitCounter  = 0;      // Number of failed messages to reattempt in each Iridium transmission (Limit: 340 bytes)

// Global variable and constant declarations
bool          ledState              = LOW;    // Flag to toggle LED in blinkLed() function

bool          rtcSyncFlag            = true;   // Flag to determine if RTC should be set using GPS time
volatile bool watchdogFlag          = false;  // Flag to indicate to Watchdog Timer if in deep sleep mode
volatile bool alarmFlag             = false;  // Flag for alarm interrupt service routine
uint8_t       resetFlag             = 0;      // Flag to force system reset using Watchdog Timer

int           valFix              = 0;      // GNSS valid fix counter
int           maxValFix           = 5;     // Maximum GNSS valid fix counter

uint8_t       transmitBuffer[340] = {};     // Qwiic Iridium 9603N transmission buffer
unsigned int  messageCounter      = 0;      // Qwiic Iridium 9603N transmitted message counter
unsigned int  retransmitCounter   = 0;      // Qwiic Iridium 9603N failed data transmission counter
unsigned int  transmitCounter     = 0;      // Qwiic Iridium 9603N transmission interval counter

unsigned long previousMillis      = 0;      // Global millis() timer

time_t        alarmTime           = 0;
time_t        unixtime            = 0;

// Union to store and send data byte-by-byte via Iridium
typedef union {
  struct {
    uint32_t  unixtime;           // UNIX Epoch time                (4 bytes)
    int32_t   latitude;           // Latitude (DD)                  (4 bytes)
    int32_t   longitude;          // Longitude (DD)                 (4 bytes)
    uint8_t   satellites;         // # of satellites                (1 byte)
    uint8_t   pdop;               // PDOP                           (1 byte)
    uint8_t   fix;                // Fix                            (1 byte)
    uint16_t  transmitDuration;   // Previous transmission duration (2 bytes)
    uint16_t  messageCounter;     // Message counter                (2 bytes)
  } __attribute__((packed));                                        // Total: (19 bytes)
  uint8_t bytes[8];
} SBDMESSAGE;

SBDMESSAGE message;
size_t messageSize = sizeof(message);   // Size (in bytes) of message to be transmitted

// Devices onboard MicroMod Data Logging Carrier Board that may be online or offline.
struct struct_online {
  bool iridium = false;
  bool gnss = false;
} online;

// Setup
void setup() {

  // Pin assignments
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  mySwitch.powerOn();

  Wire.begin(); // Initialize I2C
  //SPI1.begin(); // Initialize SPI

  Serial.begin(115200);
  //while (!Serial); // Wait for user to open Serial Monitor
  delay(5000); // Delay to allow user to open Serial Monitor

  printLine();
  Serial.println(F("Cryologger Iceberg Tracking Beacon"));
  printLine();

  configureRtc();     // Configure real-time clock (RTC)
  configureGnss();    // Configure Sparkfun SAM-M8Q
  configureIridium(); // Configure SparkFun Qwiic Iridium 9603N
  syncRtc();          // Synchronize RTC with GNSS
  configureWdt();     // Configure and start Watchdog Timer
  Serial.flush(); // Wait for transmission of any serial data to complete

}

// Loop
void loop() {

  // Check if alarm flag was set
  if (alarmFlag) {
    alarmFlag = false; // Clear alarm flag

    readRtc(); // Read RTC

    // Print date and time of RTC alarm trigger
    Serial.print("Alarm trigger: "); printDateTime();

    // Perform measurements
    petDog();       // Pet the Watchdog Timer
    //readSensors();  // Read BME280
    readGnss();     // Read GNSS
    writeBuffer();  // Write data to buffer

    // Check if data should be transmitted
    if (transmitCounter == transmitInterval) {
      transmitData();       // Transmit data
      transmitCounter = 0;  // Reset transmit counter
    }

    // Set RTC alarm
    alarmTime = unixtime + alarmInterval; // Calculate next alarm
    rtc.setAlarmEpoch(alarmTime);

    // Check if alarm was set in the past
    if (alarmTime <= rtc.getEpoch()) {
      Serial.println(F("Warning! Alarm set in the past."));
      alarmTime = rtc.getEpoch() + alarmInterval; // Calculate new alarm

      rtc.setAlarmEpoch(alarmTime);
    }
    Serial.print(F("Next alarm: ")); printAlarm();
  }
  watchdogFlag = true; // Set Watchdog Timer sleep flag
  petDog();
  blinkLed(1, 5);
}

// Blink LED (non-blocking)
void blinkLed(byte ledFlashes, unsigned int ledDelay) {

  pinMode(LED_BUILTIN, OUTPUT);
  byte i = 0;

  while (i < ledFlashes * 2) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis > ledDelay) {
      previousMillis = currentMillis;
      ledState = !ledState;
      digitalWrite(LED_BUILTIN, ledState);
      i++;
    }
  }
  digitalWrite(LED_BUILTIN, LOW);
}

// Configure the WDT to perform a system reset if loop() blocks for more than 8-16 seconds
void configureWdt() {

  // Set up the generic clock (GCLK2) used to clock the watchdog timer at 1.024kHz
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(4) |          // Divide the 32.768kHz clock source by divisor 32, where 2^(4 + 1): 32.768kHz/32=1.024kHz
                    GCLK_GENDIV_ID(2);            // Select Generic Clock (GCLK) 2
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_DIVSEL |        // Set to divide by 2^(GCLK_GENDIV_DIV(4) + 1)
                     GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK2
                     GCLK_GENCTRL_SRC_OSCULP32K | // Set the clock source to the ultra low power oscillator (OSCULP32K)
                     GCLK_GENCTRL_ID(2);          // Select GCLK2
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Feed GCLK2 to WDT (Watchdog Timer)
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK2 to the WDT
                     GCLK_CLKCTRL_GEN_GCLK2 |     // Select GCLK2
                     GCLK_CLKCTRL_ID_WDT;         // Feed the GCLK2 to the WDT
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  WDT->EWCTRL.bit.EWOFFSET = 0xA;                 // Set the Early Warning Interrupt Time Offset to 8 seconds // REG_WDT_EWCTRL = WDT_EWCTRL_EWOFFSET_8K;
  WDT->INTENSET.bit.EW = 1;                       // Enable the Early Warning interrupt                       // REG_WDT_INTENSET = WDT_INTENSET_EW;
  WDT->CONFIG.bit.PER = 0xB;                      // Set the WDT reset timeout to 16 seconds                  // REG_WDT_CONFIG = WDT_CONFIG_PER_16K;
  WDT->CTRL.bit.ENABLE = 1;                       // Enable the WDT in normal mode                            // REG_WDT_CTRL = WDT_CTRL_ENABLE;
  while (WDT->STATUS.bit.SYNCBUSY);               // Await synchronization of registers between clock domains

  // Configure and enable WDT interrupt
  NVIC_DisableIRQ(WDT_IRQn);
  NVIC_ClearPendingIRQ(WDT_IRQn);
  NVIC_SetPriority(WDT_IRQn, 0);                  // Top priority
  NVIC_EnableIRQ(WDT_IRQn);
}

// Pet the Watchdog Timer
void petDog() {
  WDT->CLEAR.bit.CLEAR = 0xA5;        // Clear the Watchdog Timer and restart time-out period //REG_WDT_CLEAR = WDT_CLEAR_CLEAR_KEY;
  while (WDT->STATUS.bit.SYNCBUSY);   // Await synchronization of registers between clock domains
}

// Watchdog Timer interrupt service routine
void WDT_Handler() {
  if (watchdogFlag) {
    watchdogFlag = false;
    WDT->INTFLAG.bit.EW = 1;          // Clear the Early Warning interrupt flag //REG_WDT_INTFLAG = WDT_INTFLAG_EW;
    WDT->CLEAR.bit.CLEAR = 0xA5;      // Clear the Watchdog Timer and restart time-out period //REG_WDT_CLEAR = WDT_CLEAR_CLEAR_KEY;
    while (WDT->STATUS.bit.SYNCBUSY); // Await synchronization of registers between clock domains
  }
  else {
    //WDT->CTRL.bit.ENABLE = 0;         // For debugging only: Disable Watchdog
    //digitalWrite(LED_BUILTIN, HIGH);  // For debugging only: Turn on LED to indicate Watchdog trigger
    while (true);                     // Force Watchdog Timer to reset the system
  }
}
