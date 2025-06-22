/*
  Title:    Cryologger Ice Tracking Beacon (ITB)
  Date:     June 22, 2025
  Author:   Adam Garbo
  Version:  3.4
  License:  GPLv3. See license file for more information.

  Components:
  - Rock7 RockBLOCK 9603
  - Adafruit Feather M0 Proto
  - Adafruit Ultimate GPS Featherwing (PA1616D)
  - Adafruit BME280 Temperature Humidity Pressure Sensor
  - Adafruit LSM6DSOX + LIS3MDL - Precision 9 DoF IMU
  - Pololu 3.3V, 600mA Step-Down Voltage Regulator D36V6F3
  - Pololu 5V, 600mA Step-Down Voltage Regulator D36V6F5
  - TN0702 N-Channel FET

  Board Definitions:
  - Arduino SAMD Boards v1.8.14
  - Adafruit SAMD Boards v1.7.16

  Comments:
  - TN0702 N-Channel FET required for On/Off operation with RockBLOCK v3.F and higher
  - Sketch uses 75904 bytes (28%) of program storage space. Maximum is 262144 bytes.
*/

// ----------------------------------------------------------------------------
// USER CONFIGURATION
// ----------------------------------------------------------------------------

// Device identifier.
#define UID "ITB_25_01"  // Unique identifier (UID)

// Alarm parameters.
#define ALARM_MODE HOURLY        // Alarm mode (MINUTE, HOURLY, DAILY)
#define ALARM_INTERVAL_DAY 0     // Alarm day interval (days)
#define ALARM_INTERVAL_HOUR 1    // Alarm hour interval (hours)
#define ALARM_INTERVAL_MINUTE 0  // Alarm minute interval (minutes)

// Transmission parameters.
#define TRANSMIT_INTERVAL 1     // Number of messages to include in each Iridium transmission (340-byte limit). Default: 1
#define TRANSMIT_REATTEMPTS 9;  // Number of attempts to retransmit failed Iridium messages (340-byte limit). Default: 3

// GNSS and Iridium parameters. Do not change unless debugging.
#define GNSS_TIMEOUT 180;     // Timeout for GNSS signal acquisition (seconds). Default: 180
#define IRIDIUM_TIMEOUT 180;  // Timeout for Iridium send/receive commands (seconds). Default: 180
#define IRIDIUM_STARTUP 120;  // Timeout for Iridium modem startup (seconds). Default: 120

// ----------------------------------------------------------------------------
//  END OF USER CONFIGURATION
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Libraries                    Version
// ----------------------------------------------------------------------------
#include <Adafruit_BME280.h>    // 2.3.0
#include <Adafruit_LIS3MDL.h>   // 1.2.4
#include <Adafruit_LSM6DSOX.h>  // 4.7.4
#include <Adafruit_Sensor.h>    // 1.1.15
#include <Arduino.h>            // Must precede <wiring_private.h>
#include <ArduinoLowPower.h>    // 1.2.2
#include <IridiumSBD.h>         // 3.0.6
#include <RTCZero.h>            // 1.6.0
#include <TimeLib.h>            // 1.6.1
#include <TinyGPS++.h>          // 1.0.3
#include <Wire.h>               //
#include <wiring_private.h>     // Required for creating new Serial instance

// ----------------------------------------------------------------------------
// Software & Hardware Versions
// ----------------------------------------------------------------------------
#define SOFTWARE_VERSION "3.4.0"
#define HARDWARE_VERSION "3.2"
#define ROCKBLOCK_VERSION_3F true

// ----------------------------------------------------------------------------
// Debugging Macros
// ----------------------------------------------------------------------------
#define DEBUG true          // Output debug messages to Serial Monitor
#define DEBUG_GNSS true     // Output GNSS debug information
#define DEBUG_IRIDIUM true  // Output Iridium debug messages to Serial Monitor

#if DEBUG
#define DEBUG_PRINT(x) SERIAL_PORT.print(x)
#define DEBUG_PRINTLN(x) SERIAL_PORT.println(x)
#define DEBUG_PRINT_HEX(x) SERIAL_PORT.print(x, HEX)
#define DEBUG_PRINTLN_HEX(x) SERIAL_PORT.println(x, HEX)
#define DEBUG_PRINT_DEC(x, y) SERIAL_PORT.print(x, y)
#define DEBUG_PRINTLN_DEC(x, y) SERIAL_PORT.println(x, y)
#define DEBUG_WRITE(x) SERIAL_PORT.write(x)

#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINT_HEX(x)
#define DEBUG_PRINTLN_HEX(x)
#define DEBUG_PRINT_DEC(x, y)
#define DEBUG_PRINTLN_DEC(x, y)
#define DEBUG_WRITE(x)
#endif

// ----------------------------------------------------------------------------
// Pin Definitions
// ----------------------------------------------------------------------------
#define PIN_VBAT A0           // Voltage divider
#define PIN_SENSOR_EN A3      // Sensor digital pin power enable
#define PIN_IMU_EN A4         // IMU digital pin enable
#define PIN_GNSS_EN A5        // GNSS enable
#define PIN_5V_EN 6           // 5V voltage regulator
#define PIN_IRIDIUM_RX 10     // RockBLOCK 9603 RXD (Yellow)
#define PIN_IRIDIUM_TX 11     // RockBLOCK 9603 TXD (Orange)
#define PIN_IRIDIUM_SLEEP 12  // RockBLOCK 9603 OnOff (Grey)

// ----------------------------------------------------------------------------
// Alarm Mode Definitions
// ----------------------------------------------------------------------------
#define MINUTE 0
#define HOURLY 1
#define DAILY 2

// ----------------------------------------------------------------------------
// Seria/UART Configuration
// ----------------------------------------------------------------------------
// Create a new UART instance on pins 10 (RX) and 11 (TX).
// More info: https://www.arduino.cc/en/Tutorial/SamdSercom
Uart Serial2(&sercom1, PIN_IRIDIUM_RX, PIN_IRIDIUM_TX,
             SERCOM_RX_PAD_2, UART_TX_PAD_0);

#define SERIAL_PORT Serial
#define GNSS_PORT Serial1
#define IRIDIUM_PORT Serial2

// Attach interrupt handler to SERCOM for new Serial instance.
void SERCOM1_Handler() {
  Serial2.IrqHandler();
}

// ----------------------------------------------------------------------------
// Object Instantiations
// ----------------------------------------------------------------------------
Adafruit_BME280 bme280;
Adafruit_LIS3MDL lis3mdl;
Adafruit_LSM6DSOX lsm6dsox;
IridiumSBD modem(IRIDIUM_PORT, PIN_IRIDIUM_SLEEP);
RTCZero rtc;
TinyGPSPlus gnss;

// Custom TinyGPS objects for fix and validity checks.
// Note: $GPGGA and $GPRMC sentences produced by GPS receivers (PA6H module)
// $GNGGA and $GNRMC sentences produced by GPS/GLONASS receivers (PA161D module)
TinyGPSCustom gnssFix(gnss, "GNGGA", 6);       // Fix quality
TinyGPSCustom gnssValidity(gnss, "GNRMC", 2);  // Validity

// ----------------------------------------------------------------------------
// Structures for Iridium SBD Transmission
// ----------------------------------------------------------------------------
// Union to store Iridium Short Burst Data (SBD) Mobile Originated (MO) messages.
typedef union {
  struct
  {
    uint32_t unixtime;          // UNIX Epoch time                (4 bytes)
    int16_t temperatureInt;     // Internal temperature (°C)      (2 bytes)   * 100
    uint16_t humidityInt;       // Internal humidity (%)          (2 bytes)   * 100
    uint16_t pressureInt;       // Internal pressure (hPa)        (2 bytes)   - 850 * 100
    int16_t pitch;              // Pitch (°)                      (2 bytes)   * 100
    int16_t roll;               // Roll (°)                       (2 bytes)   * 100
    uint16_t heading;           // Heading (°)                    (2 bytes)
    int32_t latitude;           // Latitude (DD)                  (4 bytes)   * 1000000
    int32_t longitude;          // Longitude (DD)                 (4 bytes)   * 1000000
    uint8_t satellites;         // # of satellites                (1 byte)
    uint16_t hdop;              // HDOP                           (2 bytes)
    uint16_t voltage;           // Battery voltage (V)            (2 bytes)   * 100
    uint16_t transmitDuration;  // Previous transmission duration (2 bytes)
    uint8_t transmitStatus;     // Iridium return code            (1 byte)
    uint16_t iterationCounter;  // Message counter                (2 bytes)
  } __attribute__((packed));    // Total: 34 bytes
  uint8_t bytes[34];
} SBD_MO_MESSAGE;

SBD_MO_MESSAGE moSbdMessage;

// Union to store received Iridium SBD Mobile Terminated (MT) message
typedef union {
  struct
  {
    uint8_t alarmMode;
    uint8_t alarmIntervalDay;
    uint8_t alarmIntervalHour;
    uint8_t alarmIntervalMinute;
    uint8_t transmitInterval;
    uint8_t transmitReattempts;
    uint8_t resetFlag;
  };
  uint8_t bytes[7];  // Size of message to be received in bytes
} SBD_MT_MESSAGE;

SBD_MT_MESSAGE mtSbdMessage;

// Add this line to declare the function prototype
bool validateMtSbdMessage(const SBD_MT_MESSAGE& msg);

// ----------------------------------------------------------------------------
// Structures for System Status and Timers
// ----------------------------------------------------------------------------
// Structure to store device online/offline states
struct OnlineStatus {
  bool bme280 = false;
  bool lis3mdl = false;
  bool lsm6dsox = false;
  bool gnss = false;
} online;

// Function execution timers
struct Timer {
  unsigned long readRtc;
  unsigned long readBattery;
  unsigned long readBme280;
  unsigned long readLsm6dsox;
  unsigned long readGnss;
  unsigned long iridium;
} timer;

// ------------------------------------------------------------------------------------------------
// User Defined Global Variable Declarations
// ------------------------------------------------------------------------------------------------
char uid[20] = UID;
uint8_t alarmMode = ALARM_MODE;                       // Alarm match mode
uint8_t alarmIntervalDay = ALARM_INTERVAL_DAY;        // Alarm day interval
uint8_t alarmIntervalHour = ALARM_INTERVAL_HOUR;      // Alarm hour interval
uint8_t alarmIntervalMinute = ALARM_INTERVAL_MINUTE;  // Alarm minute interval
uint8_t transmitInterval = TRANSMIT_INTERVAL;         // Messages to transmit in each Iridium transmission (340-byte limit)
uint8_t transmitReattempts = TRANSMIT_REATTEMPTS;     // Failed message transmission reattempts
unsigned int gnssTimeout = GNSS_TIMEOUT;              // Timeout for GNSS signal acquisition (seconds)
unsigned int iridiumTimeout = IRIDIUM_TIMEOUT;        // Timeout for Iridium transmission (seconds)
unsigned int IridiumStartup = IRIDIUM_STARTUP;        // Timeout for Iridium startup (seconds)

// ----------------------------------------------------------------------------
// Global Variables
// ----------------------------------------------------------------------------
// Flags
volatile bool alarmFlag = false;  // Flag for alarm interrupt service routine
volatile bool wdtFlag = false;    // Flag for Watchdog Timer interrupt service routine
bool firstTimeFlag = true;        // Flag to determine if program is running for the first time
bool resetFlag = 0;               // Flag to force system reset using Watchdog Timer

// Counters
volatile int wdtCounter = 0;        // Watchdog Timer interrupt counter
unsigned int iterationCounter = 0;  // Counter to track total number of program iterations (zero indicates a reset)

// Iridium SBD buffers and counters
uint8_t moSbdBuffer[340];           // Buffer for Mobile Originated SBD (MO-SBD) message (340 bytes max)
uint8_t mtSbdBuffer[270];           // Buffer for Mobile Terminated SBD (MT-SBD) message (270 bytes max)
size_t moSbdBufferSize;             // Size of MO-SBD message buffer
size_t mtSbdBufferSize;             // size of MT-SBD message buffer
byte transmitCounter = 0;           // Counter to track Iridium SBD transmission intervals
byte transmitReattemptCounter = 0;  // Retry attempts for failed Iridium SBD transmissions
unsigned int failureCounter = 0;    // Counter to track consecutive failed Iridium SBD transmission attempts

// RTC and timers
unsigned long unixtime = 0;        // Global epoch time variable
unsigned long alarmTime = 0;       // Global epoch alarm time variable
unsigned long gnssEpoch = 0;       // Seconds GNSS epoch time
unsigned long rtcEpoch = 0;        // Global RTC epoch time
long rtcDrift = 0;                 // Global RTC drift
tmElements_t tm;                   // Variable for converting time elements to time_t
unsigned long previousMillis = 0;  // Global millis() timer

// Measurement varaibles
float temperatureInt = 0.0;  // Internal temperature (°C)
float humidityInt = 0.0;     // Internal humidity (%)
float pressureInt = 0.0;     // Internal pressure (hPa)
float pitch = 0.0;           // Pitch (°)
float roll = 0.0;            // Roll (°)
int heading = 0;             // Tilt-compensated heading (°)
float latitude = 0.0;        // GNSS latitude (DD)
float longitude = 0.0;       // GNSS longitude (DD)
byte satellites = 0;         // GNSS satellites
float hdop = 0.0;            // GNSS HDOP
float voltage = 0.0;         // Battery voltage

// ----------------------------------------------------------------------------
// Magnetometer Min/Max Calibration
// More info:
// https://learn.adafruit.com/lsm303-accelerometer-slash-compass-breakout/calibration?view=all#calibration
// https://github.com/jremington/LSM9DS1-AHRS
//
// {1, 0, 0});    // Align to X+
// {-1, 0, 0});   // Align to X-
// {0, 1, 0});    // Align to Y+
// {0, -1, 0});   // Align to Y-
// {0, 0, 1});    // Align to Z+
// {0, 0, -1});   // Align to Z-
// ----------------------------------------------------------------------------
float p[] = { 1, 0, 0 };

// Min/max magnetometer values
float m_min[3] = {
  0, 0, 0  // Test unit
};

float m_max[3] = {
  0, 0, 0  // Test unit
};

// ----------------------------------------------------------------------------
// Setup
// ----------------------------------------------------------------------------
void setup() {
  // Pin assignments
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_5V_EN, OUTPUT);
  pinMode(PIN_GNSS_EN, OUTPUT);
  pinMode(PIN_SENSOR_EN, OUTPUT);
  pinMode(PIN_IMU_EN, OUTPUT);
  pinMode(PIN_IRIDIUM_SLEEP, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(PIN_SENSOR_EN, LOW);  // Disable power to sensors
  digitalWrite(PIN_GNSS_EN, HIGH);   // Disable power to GNSS
  digitalWrite(PIN_IMU_EN, LOW);     // Disable power to IMU
  digitalWrite(PIN_5V_EN, LOW);      // Disable power to RockBLOCK 9603
#ifdef ROCKBLOCK_VERSION_3F
  digitalWrite(PIN_IRIDIUM_SLEEP, HIGH);  // RockBLOCK v3.F and above: Set N-FET controlling RockBLOCK On/Off pin to HIGH (no voltage)
#else
  digitalWrite(PIN_IRIDIUM_SLEEP, LOW);  // RockBLOCK v3.D and below: Set On/Off pin LOW to disable power to Iridium
#endif

  // Configure analog-to-digital (ADC) converter.
  configureAdc();

  Wire.begin();           // Initialize I2C
  Wire.setClock(400000);  // Set I2C clock speed to 400 kHz

#if DEBUG
  SERIAL_PORT.begin(115200);  // Open serial port at 115200 baud
  // while (!Serial);     // Optionally wait for Serial Monitor connection
  blinkLed(4, 500);  // Non-blocking delay to allow user to open Serial Monitor
#endif

  // Output startup information
  DEBUG_PRINTLN();
  printLine();
  DEBUG_PRINTLN("Cryologger - Ice Tracking Beacon");
  printLine();
  DEBUG_PRINTLN("[Setup] Info: Initializing peripherals...");

  // Configure devices.
  configureRtc();      // Configure real-time clock (RTC)
  readRtc();           // Read datetime from RTC
  configureWdt();      // Configure Watchdog Timer (WDT)
  readBattery();       // Read battery voltage
  readGnss();          // Synchronize RTC with GNSS
  configureIridium();  // Configure RockBLOCK 9603 modem

  printSystemInfo();  // Print system information
  printSettings();    // Print configuration settings

  // Close serial port if immediately entering deep sleep
  if (!firstTimeFlag) {
    disableSerial();
  }

  // Blink LED to indicate completion of setup
  blinkLed(10, 100);
}

// ----------------------------------------------------------------------------
// Main Loop
// ----------------------------------------------------------------------------
void loop() {
  // Check if RTC alarm triggered or if program is running for the first time
  if (alarmFlag || firstTimeFlag) {
    // Read the RTC
    readRtc();

    // Reset WDT
    resetWdt();

    // Check if program is running for the first time.
    if (!firstTimeFlag) {
      // Wake from deep sleep
      wakeUp();
      blinkLed(4, 250);
    }

    DEBUG_PRINT("[Main] Info: Alarm triggered ");
    printDateTime();

    // Perform measurements
    readBattery();         // Read the battery voltage
    readGnss();            // Read the GNSS
    enableSensorPower();   // Enable power to sensor(s)
    enableImuPower();      // Enable power to IMU
    readLsm6dsox();        // Read the IMU
    readBme280();          // Read sensor(s)
    disableSensorPower();  // Disable 3.3V power to sensor(s)
    disableImuPower();     // Disable 3.3V power to IMU
    printSensors();        // Display recorded measurements
    writeBuffer();         // Write data to transmit buffer

    // Check if data transmission interval has been reached
    if ((transmitCounter == transmitInterval) || firstTimeFlag) {
      transmitData();
    }

    // Print function execution timers
    printTimers();

    // Set the next RTC alarm
    setRtcAlarm();

    DEBUG_PRINTLN("[Main] Entering deep sleep...");
    DEBUG_PRINTLN();

    // Prepare system for sleep
    prepareForSleep();
  }

  // Check for WDT interrupts
  if (wdtFlag) {
    resetWdt();
  }

  // Blink LED to indicate normal operation
  blinkLed(1, 25);

  // Enter deep sleep, awaiting RTC or WDT interrupt
  goToSleep();
}
