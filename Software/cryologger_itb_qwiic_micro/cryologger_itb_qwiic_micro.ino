/*
    Title:    Cryologger Ice Tracking Beacon (ITB) - v3.0 Prototype
    Date:     January 1, 2020
    Author:   Adam Garbo

    Description:
    - Code is currently under development for the next iteration of the Cryologger iceberg
    tracking beacon to be deployed during the 2021 Amundsen Expedition.

    Components:
    - SparkFun Qwiic Micro - SAMD21 Development Board
    - SparkFun Real Time Clock Module - RV-8803 (Qwiic)
    - SparkFun Atmospheric Sensor Breakout - BME280 (Qwiic)
    - SparkFun GPS Breakout - SAM-M8Q (Qwiic)
    - SparkFun 9DoF IMU Breakout - ICM-20948 (Qwiic)
    - Rock7 RockBLOCK 9603
    - Maxtena M1621HCT-P-SMA Iridium antenna
    - NDP6020P P-Channel MOSFET
    - SparkFun Buck-Boost Converter

    Comments:
    - The MicroMod system has potential, but requires significant hardware bug fixes.
    - Added Qwiic Power Switch and the Buck-Boost Converter feeding the 3.3 V
      bus directly, we're looking at 110-141 uA in sleep mode (total) at 7.4-12V.
    - Artemis MicroMod is currently sitting at 300 uA.
    - Initial prototype test conducted November 18, 2020.
    - December 28, 2020: Removed all SparkX components/code due to issues
*/

// -----------------------------------------------------------------------------
// Libraries
// -----------------------------------------------------------------------------
#include <Adafruit_NeoPixel.h>              // https://github.com/adafruit/Adafruit_NeoPixel
#include <ArduinoLowPower.h>                // https://github.com/arduino-libraries/ArduinoLowPower
#include <ICM_20948.h>                      // https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary
#include <IridiumSBD.h>                     // https://github.com/sparkfun/SparkFun_IridiumSBD_I2C_Arduino_Library
#include <SAMD_AnalogCorrection.h>          // https://github.com/arduino/ArduinoCore-samd/tree/master/libraries/SAMD_AnalogCorrection
#include <SparkFunBME280.h>                 // https://github.com/sparkfun/SparkFun_BME280_Arduino_Library
#include <SparkFun_RV8803.h>                // https://github.com/sparkfun/SparkFun_RV-8803_Arduino_Library
#include <SparkFun_Ublox_Arduino_Library.h> // https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library
#include <TimeLib.h>                        // https://github.com/PaulStoffregen/Time
#include <Wire.h>                           // https://www.arduino.cc/en/Reference/Wire

// -----------------------------------------------------------------------------
// Debugging macros
// -----------------------------------------------------------------------------
#define DEBUG           true   // Output debug messages to Serial Monitor
#define DEBUG_GNSS      true   // Output GNSS debug information
#define DEBUG_IRIDIUM   true   // Output Iridium debug messages to Serial Monitor

#if DEBUG
#define DEBUG_PRINT(x)            SERIAL_PORT.print(x)
#define DEBUG_PRINTLN(x)          SERIAL_PORT.println(x)
#define DEBUG_PRINT_HEX(x)        SERIAL_PORT.print(x, HEX)
#define DEBUG_PRINTLN_HEX(x)      SERIAL_PORT.println(x, HEX)
#define DEBUG_PRINT_DEC(x, y)     SERIAL_PORT.print(x, y)
#define DEBUG_PRINTLN_DEC(x, y)   SERIAL_PORT.println(x, y)
#define DEBUG_WRITE(x)            SERIAL_PORT.write(x)

#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINT_HEX(x)
#define DEBUG_PRINTLN_HEX(x)
#define DEBUG_PRINT_DEC(x, y)
#define DEBUG_PRINTLN_DEC(x, y)
#define DEBUG_WRITE(x)
#endif

// -----------------------------------------------------------------------------
// Port definitions
// -----------------------------------------------------------------------------
#define SERIAL_PORT     SerialUSB   // Required by SparkFun Qwiic Micro
#define IRIDIUM_PORT    Serial

// -----------------------------------------------------------------------------
// Pin definitions
// -----------------------------------------------------------------------------
#define PIN_VBAT            A0
#define PIN_IRIDIUM_SLEEP   4
#define PIN_LED             5
#define PIN_MOSFET          6
#define PIN_RTC_INT         7

// -----------------------------------------------------------------------------
// Object instantiations
// -----------------------------------------------------------------------------
Adafruit_NeoPixel pixels(1, PIN_LED, NEO_GRB + NEO_KHZ800);
BME280            bme280;         // I2C Address: 0x77
ICM_20948_I2C     imu;            // I2C Address: 0x69
IridiumSBD        modem(IRIDIUM_PORT, PIN_IRIDIUM_SLEEP); // D16 (TX): Pin 1 (yellow) D17 (RX): Pin 6 (orange)
RV8803            rtc;            // I2C Address: 0x32
SFE_UBLOX_GPS     gnss;           // I2C Address: 0x42

// Global constants
//const float R1 = 9973000.0;   // Voltage divider resistor 1
//const float R2 = 998400.0;    // Voltage divider resistor 2
const float R1 = 1000000.0;   // Voltage divider resistor 1
const float R2 = 1000000.0;    // Voltage divider resistor 2

// -----------------------------------------------------------------------------
// User defined global variable declarations
// -----------------------------------------------------------------------------
unsigned long alarmInterval         = 3600;   // Sleep duration in seconds
byte          alarmMinutes          = 0;      // RTC rolling alarm mintues
byte          alarmHours            = 0;      // RTC rolling alarm hours
byte          alarmDate             = 0;      // RTC rolling alarm days
byte          transmitInterval      = 1;      // Number of messages to include in each Iridium transmission (340-byte limit)
byte          retransmitCounterMax  = 4;      // Number of failed data transmissions to reattempt (340-byte limit)
unsigned long gnssDelay             = 10;    // Duration of GNSS signal acquisition (s)
unsigned long ledDelay              = 2000;   // Duration of RGB LED colour change (ms)

// -----------------------------------------------------------------------------
// Global variable declarations
// -----------------------------------------------------------------------------
volatile bool alarmFlag             = true;   // Flag for alarm interrupt service routine
volatile bool watchdogFlag          = false;  // Flag for Watchdog Timer interrupt service routine
volatile int  watchdogCounter       = 0;      // Watchdog Timer interrupt counter
bool          firstTimeFlag         = true;   // Flag to determine if the program is running for the first time
bool          rtcSyncFlag           = true;   // Flag to determine if RTC should be set using GNSS time
bool          resetFlag             = 0;      // Flag to force system reset using Watchdog Timer
byte          gnssFixCounter        = 0;      // GNSS valid fix counter
byte          gnssFixCounterMax     = 5;      // GNSS max valid fix counter
float         voltage               = 0.0;    // Battery voltage
uint8_t       transmitBuffer[340]   = {};     // Iridium 9603 transmission buffer (MO SBD message max length: 340 bytes)
unsigned int  messageCounter        = 0;      // Iridium 9603 transmission counter (zero indicates a reset)
unsigned int  retransmitCounter     = 0;      // Iridium 9603 failed transmission counter
unsigned int  transmitCounter       = 0;      // Iridium 9603 transmission interval counter
unsigned long previousMillis        = 0;      // Global millis() timer
unsigned long powerDelay            = 2500;   // Delay after power to MOSFET is enabled
time_t        alarmTime, unixtime   = 0;      // Global RTC time variables

// -----------------------------------------------------------------------------
// WS2812B RGB LED colour definitons
// -----------------------------------------------------------------------------
uint32_t white    = pixels.Color(32, 32, 32);
uint32_t red      = pixels.Color(32, 0, 0);
uint32_t green    = pixels.Color(0, 32, 0);
uint32_t blue     = pixels.Color(0, 0, 32);
uint32_t cyan     = pixels.Color(0, 32, 32);
uint32_t magenta  = pixels.Color(32, 0, 32);
uint32_t yellow   = pixels.Color(32, 32, 0);
uint32_t purple   = pixels.Color(16, 0, 32);
uint32_t orange   = pixels.Color(32, 16, 0);
uint32_t pink     = pixels.Color(32, 0, 16);
uint32_t lime     = pixels.Color(16, 32, 0);
uint32_t off      = pixels.Color(0, 0, 0);

// -----------------------------------------------------------------------------
// Data transmission unions/structures
// -----------------------------------------------------------------------------
// Union to transmit Iridium Short Burst Data (SBD) Mobile Originated (MO) message
typedef union
{
  struct
  {
    uint32_t  unixtime;           // UNIX Epoch time                (4 bytes)
    int16_t   temperature;        // Temperature (°C)               (2 bytes)
    uint16_t  humidity;           // Humidity (%)                   (2 bytes)
    uint16_t  pressure;           // Pressure (Pa)                  (2 bytes)
    int32_t   latitude;           // Latitude (DD)                  (4 bytes)
    int32_t   longitude;          // Longitude (DD)                 (4 bytes)
    uint8_t   satellites;         // # of satellites                (1 byte)
    uint16_t  pdop;               // PDOP                           (2 bytes)
    int16_t   rtcDrift;           // RTC offset from GNSS time      (2 bytes)
    uint16_t  voltage;            // Battery voltage (V)            (2 bytes)
    uint16_t  transmitDuration;   // Previous transmission duration (2 bytes)
    uint16_t  messageCounter;     // Message counter                (2 bytes)
  } __attribute__((packed));                              // Total: (29 bytes)
  uint8_t bytes[29];
} SBD_MO_MESSAGE;

SBD_MO_MESSAGE moMessage;

// Union to receive Iridium SBD Mobile Terminated (MT) message
typedef union
{
  struct
  {
    uint32_t  alarmInterval;      // (4 bytes)
    uint8_t   transmitInterval;   // (1 byte)
    uint8_t   retransmitCounter;  // (1 byte)
    uint8_t   resetFlag;          // (1 byte)
  };
  uint8_t bytes[7]; // Size of message to be received (in bytes)
} SBD_MT_MESSAGE;

SBD_MT_MESSAGE mtMessage;

// Structure to store device online/offline states
struct struct_online
{
  bool imu = false;
  bool gnss = false;
  bool iridium = false;
  bool bme280 = false;
} online;

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
void setup()
{
  // Pin assignments
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_MOSFET, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(PIN_MOSFET, HIGH);

  // Set analog resolution to 12-bits
  analogReadResolution(12);

  // Apply ADC gain and offset error calibration correction
  analogReadCorrection(17, 2057);

  Wire.begin(); // Initialize I2C
  //Wire.setClock(400000); // Set I2C clock speed to 400 kHz

  enablePower();          // Enable power to MOSFET controlled components
  configureLed();         // Configure WS2812B RGB LED

#if DEBUG
  SERIAL_PORT.begin(115200); // Begin serial at 115200 baud
  while (!SERIAL_PORT); // Wait for user to open Serial Monitor
  //blinkLed(4, 1000); // Non-blocking delay to allow user to open Serial Monitor
#endif

  setLedColour(white);

  DEBUG_PRINTLN();
  printLine();
  DEBUG_PRINTLN("Cryologger - Iceberg Tracking Beacon v3.0");
  printLine();

  // Configure devices
  configureWatchdog();    // Configure Watchdog Timer (WDT)
  configureRtc();         // Configure real-time clock (RTC)
  configureGnss();        // Configure GNSS receiver
  configureImu();         // Configure interial measurement unit (IMU)
  configureSensors();     // Configure attached sensors
  configureIridium();     // Configure Iridium 9603 transceiver
  syncRtc();              // Synchronize RTC with GNSS

  DEBUG_PRINT("Datetime: "); printDateTime();
  DEBUG_PRINT("Initial alarm: "); printAlarm();

  setLedColour(white); // Change LED colour to indicate completion of setup
}

// -----------------------------------------------------------------------------
// Loop
// -----------------------------------------------------------------------------
void loop()
{
  // Check if alarm ISR flag was set (protects against false triggers)
  if (alarmFlag)
  {
    alarmFlag = false; // Clear alarm ISR flag
    readRtc(); // Read the RTC

    // Check if RTC alarm flag was set
    if (rtc.getInterruptFlag(FLAG_ALARM) || firstTimeFlag)
    {
      if (!firstTimeFlag) {
        // Wake up
        wakeUp();
      }

      DEBUG_PRINT("Alarm interrupt trigger: ");
      printDateTime(); // Print RTC's current date and time

      // Perform measurements
      petDog();             // Reset the Watchdog Timer
      readBattery();        // Read the battery voltage
      readSensors();        // Read attached sensors
      readImu();            // Read the IMU
      readGnss();           // Read the GNSS
      writeBuffer();        // Write the data to transmit buffer
      transmitData();       // Transmit data
      setRtcAlarm();        // Set the RTC alarm
    }
  }

  // Check for Watchdog Timer interrupts
  if (watchdogFlag)
  {
    petDog(); // Reset the Watchdog Timer
  }

  // Blink LED to indicate WDT interrupt and nominal system operation
  blinkLed(1, 10);

  // Enter deep sleep and wait for WDT or RTC alarm interrupt
  goToSleep();
}
