/*
    Title:    Cryologger Ice Tracking Beacon (ITB) - Version 3
    Date:     November 18, 2020
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

// Defined constants
#define Serial  SerialUSB // Required by SparkFun Qwiic Micro 

#define DEBUG         true  // Output debugging messages to Serial Monitor
#define DIAGNOSTICS   true  // Output Iridium diagnostic messages to Serial Monitor

// Pin definitions
#define VBAT_PIN  A0

// Object instantiations
Adafruit_NeoPixel pixels(1, 4, NEO_GRB + NEO_KHZ800);
BME280            bme280;           // I2C Address: 0x77
ICM_20948_I2C     imu;              // I2C Address: 0x69
IridiumSBD        modem(Wire);      // I2C Address: 0x63
QWIIC_POWER       mySwitch;         // I2C Address: 0x41
RTCZero           rtc;
SFE_UBLOX_GPS     gps;              // I2C Address: 0x42
//SPIFlash          flash(21, &SPI1); //

// Global constant declarations
const float R1 = 9973000.0;   // Voltage divider resistor 1
const float R2 = 998400.0;    // Voltage divider resistor 2

// User defined global variable declarations
unsigned long alarmInterval         = 600;    // RTC sleep duration in seconds (Default: 3600 seconds)
byte          alarmSeconds          = 0;
byte          alarmMinutes          = 4;
byte          alarmHours            = 0;
byte          transmitInterval      = 1;     // Number of messages to transmit in each Iridium transmission (340 byte limit)
byte          maxRetransmitCounter  = 4;      // Number of failed data transmissions to reattempt (340 byte limit)

// Global variable and constant declarations
volatile bool alarmFlag             = false;  // Flag for alarm interrupt service routine
volatile bool watchdogFlag          = false;  // Flag for Watchdog Timer interrupt service routine
volatile int  watchdogCounter       = 0;      // Watchdog Timer interrupt counter
bool          ledState              = LOW;    // Flag to toggle LED in blinkLed() function
bool          rtcSyncFlag           = true;   // Flag to determine if RTC should be set using GNSS time
bool          resetFlag             = 0;      // Flag to force system reset using Watchdog Timer

int           valFix                = 0;      // GNSS valid fix counter
int           maxValFix             = 5;      // Max GNSS valid fix counter

float         voltage               = 0.0;

uint8_t       transmitBuffer[340] = {};       // Qwiic Iridium 9603N transmission buffer
unsigned int  messageCounter      = 0;        // Qwiic Iridium 9603N transmitted message counter
unsigned int  retransmitCounter   = 0;        // Qwiic Iridium 9603N failed data transmission counter
unsigned int  transmitCounter     = 0;        // Qwiic Iridium 9603N transmission interval counter

unsigned long previousMillis      = 0;        // Global millis() timer

time_t        alarmTime           = 0;
time_t        unixtime            = 0;

// NeoPixel colour definitons
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

// Union to store and send data byte-by-byte via Iridium
typedef union {
  struct {
    uint32_t  unixtime;           // UNIX Epoch time                (4 bytes)
    int16_t   temperature;        // Temperature (°C)               (2 bytes)
    uint16_t  humidity;           // Humidity (%)                   (2 bytes)
    uint32_t  pressure;           // Pressure (Pa)                  (4 bytes)
    int32_t   latitude;           // Latitude (DD)                  (4 bytes)
    int32_t   longitude;          // Longitude (DD)                 (4 bytes)
    uint8_t   satellites;         // # of satellites                (1 byte)
    uint16_t  pdop;               // PDOP                           (2 byte)
    uint16_t  voltage;            // Battery voltage (V)            (2 bytes)
    uint16_t  transmitDuration;   // Previous transmission duration (2 bytes)
    uint16_t  messageCounter;     // Message counter                (2 bytes)
  } __attribute__((packed));                                        // Total: (29 bytes)
  uint8_t bytes[19];
} SBDMESSAGE;

SBDMESSAGE message;
size_t messageSize = sizeof(message);   // Size (in bytes) of message to be transmitted

// Devices that may be online or offline.
struct struct_online {
  bool imu = false;
  bool gnss = false;
  bool iridium = false;
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
  //SPI1.begin(); // Initialize SPI

  Serial.begin(115200);
  //while (!Serial); // Wait for user to open Serial Monitor
  delay(5000); // Delay to allow user to open Serial Monitor

  Serial.println();
  printLine();
  Serial.println(F("Cryologger - Iceberg Tracking Beacon v3.0"));
  printLine();

  // Configure
  configureNeoPixel();    // Configure WS2812B RGB LED
  configureWdt();         // Configure Watchdog Timer
  configureQwiicPower();  // Configure Qwiic Power Switch
  configureRtc();         // Configure real-time clock (RTC)
  configureGnss();        // Configure Sparkfun SAM-M8Q
  configureImu();         // Configure SparkFun ICM-20948
  configureSensors();     // Configure attached sensors
  configureIridium();     // Configure SparkFun Qwiic Iridium 9603N
  syncRtc();              // Synchronize RTC with GNSS

  setPixelColour(white);
  delay(2000);
  Serial.flush(); // Wait for transmission of any serial data to complete
}

// Loop
void loop() {

  // Check if alarm flag was set
  if (alarmFlag) {
    alarmFlag = false;  // Clear alarm flag

    // Perform measurements
    //readRtc();          // Read RTC
    //printDateTime();
    Serial.print(F("UNIX Epoch time: ")); Serial.println(unixtime);
    petDog();           // Pet the Watchdog Timer
    readBattery();
    readSensors();      // Read sensors
    readImu();          // Read IMU
    readGnss();         // Read GNSS
    qwiicPowerOff();    // Disable power to Qwiic devices
    writeBuffer();      // Write data to buffer
    transmitData();     // Transmit data
    setRtcAlarm();      // Set RTC alarm
  }

  // Check for watchdog interrupt
  if (watchdogFlag) {
    petDog();
  }

  // Blink LED
  blinkLed(1, 50);

  // Enter deep sleep and await WDT or RTC alarm interrupt
  goToSleep();
}
