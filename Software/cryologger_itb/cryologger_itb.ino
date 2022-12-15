/*
  Title:    Cryologger Ice Tracking Beacon (ITB) - v3.2.0
  Date:     December 15, 2022
  Author:   Adam Garbo

  Description:
  - Code intended for deployments to be made during the 2022 Grise Fiord community visit.

  Components:
  - Rock7 RockBLOCK 9603
  - Maxtena M1621HCT-P-SMA antenna
  - Adafruit Feather M0 Proto
  - Adafruit Ultimate GPS Featherwing (PA1616D)
  - Adafruit BME280 Temperature Humidity Pressure Sensor
  - Adafruit LSM303AGR Accelerometer/Magnetomter
  - Pololu 3.3V, 600mA Step-Down Voltage Regulator D36V6F3
  - Pololu 5V, 600mA Step-Down Voltage Regulator D36V6F5

  Comments:
  - Began merging of code with the Cryologger Automatic Weather Station (AWS) repository.
  - The LSM6DS33/LIS3MDL 9-dof IMU was discontinued and replaced with the LSM303AGR/LIS2MDL 6-dof
  accelerometer/magnetometer.
*/

// ------------------------------------------------------------------------------------------------
// Libraries
// ------------------------------------------------------------------------------------------------
#include <Adafruit_BME280.h>        // https://github.com/adafruit/Adafruit_BME280 (v2.2.2)
#include <Adafruit_DPS310.h>        // https://github.com/adafruit/Adafruit_DPS310 (v1.1.1)
#include <Adafruit_LSM303_Accel.h>  // https://github.com/adafruit/Adafruit_LSM303_Accel (v1.1.6)
#include <Adafruit_LIS2MDL.h>       // https://github.com/adafruit/Adafruit_LIS2MDL (v2.1.4)
#include <Adafruit_Sensor.h>        // https://github.com/adafruit/Adafruit_Sensor (v1.1.6)
#include <Arduino.h>                // Required for creating new Serial instance. Must be included before <wiring_private.h>
#include <ArduinoLowPower.h>        // https://github.com/arduino-libraries/ArduinoLowPower (v1.2.2)
#include <IridiumSBD.h>             // https://github.com/sparkfun/SparkFun_IridiumSBD_I2C_Arduino_Library (v3.0.5)
#include <RTCZero.h>                // https://github.com/arduino-libraries/RTCZero (v1.6.0)
#include <TimeLib.h>                // https://github.com/PaulStoffregen/Time (v1.6.1)
#include <TinyGPS++.h>              // https://github.com/mikalhart/TinyGPSPlus (v1.0.3)
#include <Wire.h>                   // https://www.arduino.cc/en/Reference/Wire
#include <wiring_private.h>         // Required for creating new Serial instance

// ----------------------------------------------------------------------------
// Define unique identifier
// ----------------------------------------------------------------------------
#define CRYOLOGGER_ID 1

// ------------------------------------------------------------------------------------------------
// Debugging macros
// ------------------------------------------------------------------------------------------------
#define DEBUG           true  // Output debug messages to Serial Monitor
#define DEBUG_GNSS      true  // Output GNSS debug information
#define DEBUG_IRIDIUM   true  // Output Iridium debug messages to Serial Monitor

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

// ------------------------------------------------------------------------------------------------
// Pin definitions
// ------------------------------------------------------------------------------------------------
#define PIN_VBAT            A0
#define PIN_SENSOR_EN       A3
#define PIN_IMU_EN          A4
#define PIN_GNSS_EN         A5
#define PIN_LED             5
#define PIN_IRIDIUM_EN      6
#define PIN_IRIDIUM_RX      10 // Pin 1 RXD (Yellow)
#define PIN_IRIDIUM_TX      11 // Pin 6 TXD (Orange)
#define PIN_IRIDIUM_SLEEP   12 // Pin 7 OnOff (Grey)

// ------------------------------------------------------------------------------------------------
// Port configuration
// ------------------------------------------------------------------------------------------------
// Create a new UART instance and assign it to pins 10 (RX) and 11 (TX).
// For more information see: https://www.arduino.cc/en/Tutorial/SamdSercom
Uart Serial2 (&sercom1, PIN_IRIDIUM_RX, PIN_IRIDIUM_TX, SERCOM_RX_PAD_2, UART_TX_PAD_0);

#define SERIAL_PORT   Serial
#define GNSS_PORT     Serial1
#define IRIDIUM_PORT  Serial2

// Attach interrupt handler to SERCOM for new Serial instance
void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}

// ------------------------------------------------------------------------------------------------
// Object instantiations
// ------------------------------------------------------------------------------------------------
Adafruit_BME280   bme280;   // I2C address: 0x??
Adafruit_DPS310   dps310;   // I2C address: 0x77
Adafruit_LIS2MDL  lis2mdl = Adafruit_LIS2MDL(12345); //
Adafruit_LSM303_Accel_Unified   lsm303agr = Adafruit_LSM303_Accel_Unified(54321); // I2C address: 0x1E
IridiumSBD        modem(IRIDIUM_PORT, PIN_IRIDIUM_SLEEP);
RTCZero           rtc;
TinyGPSPlus       gnss;

// Custom TinyGPS objects to store fix and validity information
// Note: $GPGGA and $GPRMC sentences are GPS only (PA6H module)
// $GNGGA and $GNRMC sentences are produced by GPS/GLONASS receivers (PA161D module)
TinyGPSCustom gnssFix(gnss, "GNGGA", 6); // Fix quality
TinyGPSCustom gnssValidity(gnss, "GNRMC", 2); // Validity

// ------------------------------------------------------------------------------------------------
// User defined global variable declarations
// ------------------------------------------------------------------------------------------------

unsigned long alarmInterval     = 3600;   // Sleep duration (seconds)
unsigned int  transmitInterval  = 1;      // Messages to transmit in each Iridium transmission (340 byte limit)
unsigned int  retransmitLimit   = 6;      // Failed data transmission reattempt (340-byte limit)
unsigned int  gnssTimeout       = 2;      // Timeout for GNSS signal acquisition (minutes)
unsigned int  iridiumTimeout    = 180;    // Timeout for Iridium transmission (seconds)
bool          firstTimeFlag     = true;   // Flag to determine if the program is running for the first time

// ------------------------------------------------------------------------------------------------
// Global variable declarations
// ------------------------------------------------------------------------------------------------
volatile bool alarmFlag         = false;      // Flag for alarm interrupt service routine
volatile bool wdtFlag           = false;      // Flag for Watchdog Timer interrupt service routine
volatile int  wdtCounter        = 0;          // Watchdog Timer interrupt counter
bool          resetFlag         = 0;          // Flag to force system reset using Watchdog Timer
uint8_t       moSbdBuffer[340];               // Buffer for Mobile Originated SBD (MO-SBD) message (340 bytes max)
uint8_t       mtSbdBuffer[270];               // Buffer for Mobile Terminated SBD (MT-SBD) message (270 bytes max)
size_t        moSbdBufferSize;
size_t        mtSbdBufferSize;
unsigned int  iterationCounter  = 0;          // Counter to track total number of program iterations (zero indicates a reset)
byte          samples           = 30;         // Number of samples to average accelerometer and magnetometer readings
byte          retransmitCounter = 0;          // Counter to track Iridium 9603 transmission reattempts
byte          transmitCounter   = 0;          // Counter to track Iridium 9603 transmission intervals
unsigned int  failureCounter    = 0;          // Counter to track consecutive failed Iridium transmission attempts
unsigned long previousMillis    = 0;          // Global millis() timer
unsigned long alarmTime         = 0;          // Global epoch alarm time variable
unsigned long unixtime          = 0;          // Global epoch time variable
float         temperatureInt    = 0.0;        // Internal temperature (°C)
float         humidityInt       = 0.0;        // Internal hunidity (%)
float         pressureInt       = 0.0;        // Internal pressure (hPa)
float         voltage           = 0.0;        // Battery voltage
tmElements_t  tm;                             // Variable for converting time elements to time_t

// ------------------------------------------------------------------------------------------------
// Magnetometer min/max calibration
// ------------------------------------------------------------------------------------------------
// Based on:
// https://learn.adafruit.com/lsm303-accelerometer-slash-compass-breakout/calibration?view=all#calibration
// https://gist.github.com/CalebFenton/a97444750eb43e3354fd2d0196a2ebcf
// https://github.com/jremington/LSM9DS1-AHRS

float p[] = {1, 0, 0};  // X marking on sensor board points toward yaw = 0 (N?)

// Min/max magnetometer values
float m_min[3] = {
  -75.60, -62.25, -52.35
};

float m_max[3] = {
  51.75, 72.60, 78.30
};

// ------------------------------------------------------------------------------------------------
// Data transmission unions/structures
// ------------------------------------------------------------------------------------------------
// Union to store Iridium Short Burst Data (SBD) Mobile Originated (MO) messages
typedef union
{
  struct
  {
    uint32_t  unixtime;         // UNIX Epoch time                (4 bytes)
    int16_t   temperatureInt;   // Internal temperature (°C)      (2 bytes)   * 100
    uint16_t  humidityInt;      // Internal humidity (%)          (2 bytes)   * 100
    uint16_t  pressureInt;      // Internal pressure (hPa)        (2 bytes)   - 850 * 100
    int16_t   pitch;            // Pitch (°)                      (2 bytes)   * 100
    int16_t   roll;             // Roll (°)                       (2 bytes)   * 100
    uint16_t  heading;          // Heading (°)                    (2 bytes)
    int32_t   latitude;         // Latitude (DD)                  (4 bytes)   * 1000000
    int32_t   longitude;        // Longitude (DD)                 (4 bytes)   * 1000000
    uint8_t   satellites;       // # of satellites                (1 byte)
    uint16_t  hdop;             // HDOP                           (2 bytes)
    uint16_t  voltage;          // Battery voltage (V)            (2 bytes)   * 100
    uint16_t  transmitDuration; // Previous transmission duration (2 bytes)
    uint8_t   transmitStatus;   // Iridium return code            (1 byte)
    uint16_t  iterationCounter; // Message counter                (2 bytes)
  } __attribute__((packed));                            // Total: (34 bytes)
  uint8_t bytes[34];
} SBD_MO_MESSAGE;

SBD_MO_MESSAGE moSbdMessage;

// Union to store received Iridium SBD Mobile Terminated (MT) message
typedef union
{
  struct
  {
    uint32_t  alarmInterval;      // 4 bytes
    uint8_t   transmitInterval;   // 1 byte
    uint8_t   retransmitLimit;    // 1 byte
    uint8_t   resetFlag;          // 1 byte
  };
  uint8_t bytes[7]; // Size of message to be received in bytes
} SBD_MT_MESSAGE;

SBD_MT_MESSAGE mtSbdMessage;

// Structure to store device online/offline states
struct struct_online
{
  bool bme280     = false;
  bool dps310     = false;
  bool lis2mdl    = false;
  bool lsm303agr  = false;
  bool gnss       = false;
} online;

// Union to store loop timers`
struct struct_timer
{
  unsigned long readRtc;
  unsigned long readBattery;
  unsigned long readBme280;
  unsigned long readDps310;
  unsigned long readLsm303agr;
  unsigned long readGnss;
  unsigned long iridium;

} timer;

// ------------------------------------------------------------------------------------------------
// Setup
// ------------------------------------------------------------------------------------------------
void setup()
{
  // Pin assignments
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_IRIDIUM_EN, OUTPUT);
  pinMode(PIN_GNSS_EN, OUTPUT);
  pinMode(PIN_SENSOR_EN, OUTPUT);
  pinMode(PIN_IMU_EN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(PIN_SENSOR_EN, LOW);   // Disable power to sensors
  digitalWrite(PIN_GNSS_EN, HIGH);    // Disable power to GNSS
  digitalWrite(PIN_IMU_EN, LOW);      // Disable power to IMU
  digitalWrite(PIN_IRIDIUM_EN, LOW);  // Disable power to Iridium 9603

  // Configure analog-to-digital (ADC) converter
  configureAdc();

  Wire.begin(); // Initialize I2C
  Wire.setClock(400000); // Set I2C clock speed to 400 kHz

#if DEBUG
  SERIAL_PORT.begin(115200); // Open serial port at 115200 baud
  blinkLed(LED_BUILTIN, 4, 1000); // Non-blocking delay to allow user to open Serial Monitor
#endif

  DEBUG_PRINTLN();
  printLine();
  DEBUG_PRINT("Cryologger Ice Tracking Beacon - Grise Fiord #"); DEBUG_PRINTLN(CRYOLOGGER_ID);
  printLine();

  // Configure devices
  configureRtc();       // Configure real-time clock (RTC)
  readRtc();            // Read date and time from RTC
  configureWdt();       // Configure Watchdog Timer (WDT)
  readBattery();        // Read battery at start-up
  printSettings();      // Print configuration settings
  readGnss();           // Synchronize RTC with GNSS
  configureIridium();   // Configure Iridium 9603 transceiver

  // Close serial port if immediately entering deep sleep
  if (!firstTimeFlag)
  {
    disableSerial();
  }

  // Blink LED to indicate completion of setup
  blinkLed(LED_BUILTIN, 10, 100);
}

// ------------------------------------------------------------------------------------------------
// Loop
// ------------------------------------------------------------------------------------------------
void loop()
{
  // Check if RTC alarm triggered or if program is running for first time
  if (alarmFlag || firstTimeFlag)
  {
    // Read the RTC
    readRtc();

    // Reset WDT
    petDog();

    // Check if program is running for the first time
    if (!firstTimeFlag)
    {
      wakeUp();
    }

    DEBUG_PRINT("Info: Alarm trigger "); printDateTime();

    // Perform measurements
    enableSensorPower();  // Enable 3.3V power to sensor(s)
    enableImuPower();     // Enable 3.3V power to IMU
    readBattery();        // Read the battery voltage
    readGnss();           // Read the GNSS
    readLsm303agr();         // Read the IMU
    readBme280();         // Read sensor(s)
    disableSensorPower(); // Disable 3.3V power to sensor(s)
    disableImuPower();    // Disable 3.3V power to IMU

    writeBuffer();        // Write the data to transmit buffer
    transmitData();       // Transmit data via Iridium transceiver
    printTimers();        // Print function execution timers
    setRtcAlarm();        // Set the RTC alarm

    DEBUG_PRINTLN("Info: Entering deep sleep...");
    DEBUG_PRINTLN();

    // Prepare for sleep
    prepareForSleep();
  }

  // Check for Watchdog Timer interrupts
  if (wdtFlag)
  {
    petDog(); // Reset the Watchdog Timer
  }

  // Blink LED to indicate WDT interrupt and nominal system operation
  blinkLed(LED_BUILTIN, 1, 25);

  // Enter deep sleep and wait for WDT or RTC alarm interrupt
  goToSleep();
}
