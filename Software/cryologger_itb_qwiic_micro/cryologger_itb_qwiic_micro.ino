/*
    Title:    Cryologger Ice Tracking Beacon (ITB) - Version 3
    Date:     November 30, 2020
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
    - Initial prototype test conducted November 18, 2020.
*/

// Libraries
#include <Wire.h>                                         // https://www.arduino.cc/en/Reference/Wire
//#include <SPIMemory.h>                                    // https://github.com/Marzogh/SPIMemory
#include <TimeLib.h>                                      // https://github.com/PaulStoffregen/Time
#include <SparkFun_Ublox_Arduino_Library.h>               // https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library
#include <SparkFun_Qwiic_Power_Switch_Arduino_Library.h>  // https://github.com/sparkfun/SparkFun_Qwiic_Power_Switch_Arduino_Library
#include <SparkFunBME280.h>                               // https://github.com/sparkfun/SparkFun_BME280_Arduino_Library
#include <SAMD_AnalogCorrection.h>                        // https://github.com/arduino/ArduinoCore-samd/tree/master/libraries/SAMD_AnalogCorrection
#include <RTCZero.h>                                      // https://github.com/arduino-libraries/RTCZero
#include <IridiumSBD.h>                                   // https://github.com/sparkfun/SparkFun_IridiumSBD_I2C_Arduino_Library
#include <ICM_20948.h>                                    // https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary
#include <ArduinoLowPower.h>                              // https://github.com/arduino-libraries/ArduinoLowPower
#include <Adafruit_NeoPixel.h>                            // https://github.com/adafruit/Adafruit_NeoPixel

// Port definitions
#define SERIAL_PORT     SerialUSB   // Required by SparkFun Qwiic Micro
#define IRIDIUM_PORT    Serial      
#define IRIDIUM_WIRE    Wire

// Debugging definitions
#define DEBUG           true        // Output debug messages to Serial Monitor
#define DEBUG_GNSS      true        // Output GNSS debug information
#define DEBUG_IRIDIUM   true        // Output Iridium diagnostic messages to Serial Monitor

// Pin definitions
#define VBAT_PIN            A1
#define IRIDIUM_SLEEP_PIN   7

// Object instantiations
Adafruit_NeoPixel pixels(1, 4, NEO_GRB + NEO_KHZ800);
BME280            bme280;         // I2C Address: 0x77
ICM_20948_I2C     imu;            // I2C Address: 0x69
//IridiumSBD        modem(IRIDIUM_WIRE); // I2C Address: 0x63
IridiumSBD        modem(IRIDIUM_PORT, IRIDIUM_SLEEP_PIN); // D16: Pin 1 (yellow) D17: Pin 6 (orange)
QWIIC_POWER       mySwitch;       // I2C Address: 0x41
RTCZero           rtc;
SFE_UBLOX_GPS     gps;            // I2C Address: 0x42
//SPIFlash          flash(21, &SPI1); //

// Global constants
const float R1 = 9973000.0;   // Voltage divider resistor 1
const float R2 = 998400.0;    // Voltage divider resistor 2

// User defined global variables
unsigned long alarmInterval         = 3600;    // Sleep duration in seconds (Default: 3600 seconds)
byte          alarmSeconds          = 0;
byte          alarmMinutes          = 5;
byte          alarmHours            = 0;
byte          transmitInterval      = 1;      // Number of messages to include in each Iridium transmission (340-byte limit)
byte          maxRetransmitCounter  = 0;      // Number of failed data transmissions to reattempt (340-byte limit)

// Global variables
volatile bool alarmFlag             = true;   // Flag for alarm interrupt service routine
volatile bool watchdogFlag          = false;  // Flag for Watchdog Timer interrupt service routine
volatile int  watchdogCounter       = 0;      // Watchdog Timer interrupt counter
bool          firstTimeFlag         = true;   // Flag to determine if the program is running for the first time
bool          ledStateFlag          = LOW;    // Flag to toggle LED in blinkLed() function
bool          rtcSyncFlag           = true;   // Flag to determine if RTC should be set using GNSS time
bool          resetFlag             = 0;      // Flag to force system reset using Watchdog Timer

byte          gnssFixCounter        = 0;      // GNSS valid fix counter
byte          gnssFixCounterMax     = 5;      // GNSS max valid fix counter

float         voltage               = 0.0;

uint8_t       transmitBuffer[340]   = {};     // Iridium 9603 transmission buffer (MO SBD message max length: 340 bytes)
unsigned int  messageCounter        = 0;      // Iridium 9603 cumualtive transmission counter (zero indicates a reset)
unsigned int  retransmitCounter     = 0;      // Iridium 9603 failed transmission counter
unsigned int  transmitCounter       = 0;      // Iridium 9603 transmission interval counter

unsigned long previousMillis        = 0;      // Global millis() timer

time_t        alarmTime, unixtime   = 0;      // Global RTC time variables


// WS2812B RGB LED colour definitons
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

// Union to store and send data byte-by-byte via Iridium
typedef union {
  struct {
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
  } __attribute__((packed));                                        // Total: (29 bytes)
  uint8_t bytes[29];
} SBDMESSAGE;

SBDMESSAGE message;
size_t messageSize = sizeof(message);   // Size (in bytes) of message to be transmitted

// Devices that may be online or offline.
struct struct_online {
  bool imu = false;
  bool gnss = false;
  bool iridium = false;
  bool powerSwitch = false;
  bool bme280 = false;
} online;

// Setup
void setup() {

  // Pin assignments
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Set analog resolution to 12-bits
  analogReadResolution(12);
  // Apply ADC gain and offset error calibration correction
  analogReadCorrection(17, 2057);

  Wire.begin(); // Initialize I2C
  Wire.setClock(400000); // Set I2C clock speed to 400 kHz
  //SPI1.begin(); // Initialize SPI

  SERIAL_PORT.begin(115200); // Begin serial at 115200 baud
  //while (!SERIAL_PORT); // Wait for user to open Serial Monitor
  blinkLed(4, 1000); // Non-blocking delay to allow user to open Serial Monitor

  SERIAL_PORT.println();
  printLine();
  SERIAL_PORT.println(F("Cryologger - Iceberg Tracking Beacon v3.0"));
  printLine();

  // Configuration
  configureQwiicPower();  // Configure Qwiic Power Switch
  qwiicPowerOn();         // Enable power to Qwiic peripherals
  configureLed();         // Configure WS2812B RGB LED
  configureWatchdog();    // Configure Watchdog Timer
  configureRtc();         // Configure real-time clock (RTC)
  configureGnss();        // Configure Sparkfun SAM-M8Q
  configureImu();         // Configure SparkFun ICM-20948
  configureSensors();     // Configure attached sensors
  //configureIridiumI2C();  // Configure SparkFun Qwiic Iridium 9603N
  configureIridium();     // Configure RockBLOCK Iridium 9603N
  //syncRtc();              // Synchronize RTC with GNSS

  setLedColour(white);
}

// Loop
void loop() {

  // Check if alarm flag was set
  if (alarmFlag) {
    alarmFlag = false; // Clear alarm flag
    printDateTime(); // Print RTC's current date and time

    // Perform measurements
    readRtc();
    petDog();             // Reset the Watchdog Timer
    readBattery();        // Read the battery voltage
    readSensors();        // Read attached sensors
    readImu();            // Read the IMU
    readGnss();           // Read the GNSS
    writeBuffer();        // Write the data to transmit buffer
    //transmitDataI2C();    // Transmit data
    transmitData();       // Transmit data
    qwiicPowerOff();      // Disable power to Qwiic peripheral devices
    setRtcAlarm();        // Set the RTC alarm
  }

  // Check for watchdog interrupt
  if (watchdogFlag) {
    petDog(); // Reset the Watchdog Timer
  }

  // Blink LED
  blinkLed(1, 25);

  // Enter deep sleep and wait for WDT or RTC alarm interrupt
  goToSleep();
}
