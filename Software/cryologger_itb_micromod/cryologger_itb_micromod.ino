/*
    Title:    Cryologger Ice Tracking Beacon (ITB) - Version 3
    Date:     November 10, 2020
    Author:   Adam Garbo

    Components:
    - SparkFun Artemis Processor
    - SparkFun MicroMod Data Logging Carrier Board
    - SparkFun GPS Breakout - SAM-M8Q (Qwiic)
    - SparkFun Qwiic Iridium 9603N
    - Maxtena M1621HCT-P-SMA Iridium antenna
    - SparkFun Buck-Boost Converter

    Comments:

*/

// Libraries
#include <IridiumSBD.h>                     // http://librarymanager/All#IridiumSBDI2C
#include <RTC.h>
#include <SparkFun_Ublox_Arduino_Library.h> // http://librarymanager/All#SparkFun_Ublox_GPS
#include <SdFat.h>                          // http://librarymanager/All#SdFat
#include <SPI.h>
#include <WDT.h>
#include <Wire.h>

// Defined constants
#define DEBUG         true    // Output debug messages to Serial Monitor
#define DIAGNOSTICS   false    // Output Iridium diagnostic messages to Serial Monitor

// Firmware version
#define FIRMWARE_VERSION_MAJOR 3
#define FIRMWARE_VERSION_MINOR 0

// Pin definitions
#define PIN_PWC_POWER           G1
#define PIN_QWIIC_POWER         G2
#define PIN_MICROSD_CHIP_SELECT 41

// Object instantiations
APM3_RTC      rtc;
APM3_WDT      wdt;
IridiumSBD    modem(Wire);    // I2C Address: 0x63
SdFat         sd;             // File system object
SdFile        file;           // Log file
SFE_UBLOX_GPS gps;            // I2C Address: 0x42

// User defined global variable declarations
byte          alarmSeconds          = 0;
byte          alarmMinutes          = 1;
byte          alarmHours            = 0;
unsigned int  transmitInterval      = 10;    // Messages included in each RockBLOCK transmission (340 byte limit)
unsigned int  maxRetransmitCounter  = 0;    // Max failed data transmissions reattempts in a single message (340 byte limit)

// Global variable and constant declarations
volatile bool alarmFlag           = false;  // RTC ISR flag
volatile bool watchdogFlag        = false;  // Watchdog Timer ISR flag
volatile int  watchdogCounter     = 0;      // Watchdog interrupt counter
bool          ledState            = LOW;    // LED flag for blinkLed()
bool          rtcSyncFlag         = false;

int           valFix              = 0;      // GNSS valid fix counter
int           maxValFix           = 1;      // Max GNSS valid fix counter

char          fileName[30]        = "";     // Keep a record of this file name so that it can be re-opened upon wakeup from sleep
char          outputData[512];              // Factor of 512 for easier recording to SD in 512 chunks
char          tempData[100];                // Temporary SD data buffer
unsigned int  sdPowerDelay        = 100;    // Delay (in milliseconds) before disabling power to microSD
unsigned int  qwiicPowerDelay     = 1000;   // Delay (in milliseconds) after enabling power to Qwiic connector

uint8_t       transmitBuffer[340] = {};     // Qwiic Iridium 9603N transmission buffer
unsigned int  messageCounter      = 0;      // Qwiic Iridium 9603N transmitted message counter
unsigned int  retransmitCounter   = 0;      // Qwiic Iridium 9603N failed data transmission counter
unsigned int  transmitCounter     = 0;      // Qwiic Iridium 9603N transmission interval counter

unsigned long previousMillis      = 0;      // Global millis() timer

// Union to store and send data byte-by-byte via Iridium
typedef union {
  struct {
    uint32_t  unixtime;           // UNIX Epoch time                (4 bytes)
    int32_t   latitude;           // Latitude (DD)                  (4 bytes)
    int32_t   longitude;          // Longitude (DD)                 (4 bytes)
    uint8_t   satellites;         // # of satellites                (1 byte)
    uint8_t   fix;                // Fix                            (1 byte)
    uint8_t   pdop;               // PDOP                           (1 byte)
    uint16_t  transmitDuration;   // Previous transmission duration (2 bytes)
    uint16_t  messageCounter;     // Message counter                (2 bytes)
  } __attribute__((packed));                                        // Total: (19 bytes)
  uint8_t bytes[8];
} SBDMESSAGE;

SBDMESSAGE message;
size_t messageSize = sizeof(message);   // Size (in bytes) of message to be transmitted

// Devices onboard MicroMod Data Logging Carrier Board that may be online or offline.
struct struct_online {
  bool microSd = false;
  bool iridium = false;
  bool gnss = false;
} online;

void setup() {

  // Pin assignments
  pinMode(PIN_QWIIC_POWER, OUTPUT);
  pinMode(PIN_PWC_POWER, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  qwiicPowerOn(); // Enable power to Qwiic connector
  peripheralPowerOn(); // Enable power to peripherials

  Wire.begin(); // Initialize I2C
  SPI.begin(); // Initialize SPI

  Serial.begin(115200);
  //while (!Serial); // Wait for user to open Serial Monitor
  delay(5000); // Delay to allow user to open Serial Monitor

  printLine();
  Serial.printf("Cryologger Iceberg Tracking Beacon v%d.%d\n", FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR);
  printLine();

  Serial.print(F("Datetime: ")); printDateTime();

  configureSd();      // Configure microSD
  configureGnss();    // Configure Sparkfun SAM-M8Q
  configureIridium(); // Configure SparkFun Qwiic Iridium 9603N
  configureRtc();     // Configure real-time clock (RTC)
  //syncRtc();          // Synchronize RTC with GNSS
  configureWdt();     // Configure and start Watchdog Timer
  createLogFile();
  Serial.flush(); // Wait for transmission of any serial data to complete
}

void loop() {

  // Check if alarm flag was set
  if (alarmFlag) {
    alarmFlag = false; // Clear alarm flag

    // Print date and time of RTC alarm trigger
    Serial.print("Alarm trigger: "); printDateTime();

    readRtc(); // Read RTC

    readGnss(); // Read GNSS
    writeBuffer(); // Write data to buffer
    logData(); // Write data to SD

    // Check if data is to be transmitted
    if (transmitCounter == transmitInterval) {
      transmitData(); // Transmit data
      transmitCounter = 0; // Reset transmit counter
    }
    setRtcAlarm();
  }

  // Check for watchdog interrupt
  if (watchdogFlag) {
    petDog();
  }

  // Blink LED
  blinkLed(1, 500);

  // Enter deep sleep and await RTC alarm interrupt
  //goToSleep();
}

// Interrupt handler for the RTC
extern "C" void am_rtc_isr(void) {

  // Clear the RTC alarm interrupt
  am_hal_rtc_int_clear(AM_HAL_RTC_INT_ALM);

  // Set alarm flag
  alarmFlag = true;
}

// Interrupt handler for the watchdog.
extern "C" void am_watchdog_isr(void) {

  // Clear the watchdog interrupt
  wdt.clear();

  // Perform system reset after 15 watchdog interrupts (should not occur)
  if (watchdogCounter < 25 ) {
    wdt.restart(); // "Pet" the dog
  }
  watchdogFlag = true; // Set the watchdog flag
  watchdogCounter++; // Increment watchdog interrupt counter
}
