/*
  Title:    Cryologger Ice Tracking Beacon (ITB) - v3.2.0
  Date:     April 15, 2023
  Author:   Adam Garbo

  Components:
  - Rock7 RockBLOCK 9603
  - Adafruit Feather M0 Proto
  - Adafruit Ultimate GPS Featherwing (PA1616D)
  - Adafruit BME280 Temperature Humidity Pressure Sensor
  - Adafruit LSM303AGR Accelerometer/Magnetomter
  - Pololu 3.3V, 600mA Step-Down Voltage Regulator D36V6F3
  - Pololu 5V, 600mA Step-Down Voltage Regulator D36V6F5

  Comments:
  - Intended for deployment during the 2023 Amundsen Expedition.
*/

// ------------------------------------------------------------------------------------------------
// Libraries
// ------------------------------------------------------------------------------------------------
#include <Adafruit_BME280.h>        // http://librarymanager/All#Adafruit_BME280 (v2.2.2)
#include <Adafruit_LSM303_Accel.h>  // http://librarymanager/All#Adafruit_LSM303_Accel (v1.1.6)
#include <Adafruit_LIS2MDL.h>       // http://librarymanager/All#Adafruit_LIS2MDL (v2.1.4)
#include <Adafruit_Sensor.h>        // http://librarymanager/All#Adafruit_Sensor (v1.1.6)
#include <Arduino.h>                // Required for creating new Serial instance. Must be included before <wiring_private.h>
#include <ArduinoLowPower.h>        // http://librarymanager/All#ArduinoLowPower (v1.2.2)
#include <IridiumSBD.h>             // http://librarymanager/All#SparkFun_IridiumSBD_I2C_Arduino_Library (v3.0.5)
#include <RTCZero.h>                // http://librarymanager/All#RTCZero (v1.6.0)
#include <TimeLib.h>                // http://librarymanager/All#Timelib (v1.6.1)
#include <TinyGPS++.h>              // http://librarymanager/All#TinyGPSPlus (v1.0.3)
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
#define PIN_5V_EN           6
#define PIN_IRIDIUM_RX      10 // Pin 1 RXD (Yellow)
#define PIN_IRIDIUM_TX      11 // Pin 6 TXD (Orange)
#define PIN_IRIDIUM_SLEEP   12 // Pin 7 OnOff (Grey)
#define PIN_LED_RED         13

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

// ----------------------------------------------------------------------------
// Object instantiations
// ----------------------------------------------------------------------------
Adafruit_BME280                 bme280; // I2C address:
Adafruit_LIS2MDL                lis2mdl = Adafruit_LIS2MDL(12345); // I2C address:
Adafruit_LSM303_Accel_Unified   lsm303 = Adafruit_LSM303_Accel_Unified(54321); // I2C address: 0x1E
IridiumSBD                      modem(IRIDIUM_PORT, PIN_IRIDIUM_SLEEP);
RTCZero                         rtc;
TinyGPSPlus                     gnss;

// Custom TinyGPS objects to store fix and validity information
// Note: $GPGGA and $GPRMC sentences produced by GPS receivers (PA6H module)
// $GNGGA and $GNRMC sentences produced by GPS/GLONASS receivers (PA161D module)
TinyGPSCustom gnssFix(gnss, "GNGGA", 6); // Fix quality
TinyGPSCustom gnssValidity(gnss, "GNRMC", 2); // Validity

// ------------------------------------------------------------------------------------------------
// User defined global variable declarations
// ------------------------------------------------------------------------------------------------
unsigned long sampleInterval    = 60;     // Sampling interval (minutes). Default: 60 minutes (1 hour)
unsigned int  averageInterval   = 1;      // Number of samples to be averaged in each transmission. Default: 1 (hourly)
unsigned int  transmitInterval  = 3;      // Messages to transmit in each Iridium transmission (340 byte limit)
unsigned int  retransmitLimit   = 3;      // Failed data transmission reattempt (340-byte limit)
unsigned int  gnssTimeout       = 1;    // Timeout for GNSS signal acquisition (seconds)
unsigned int  iridiumTimeout    = 30;    // Timeout for Iridium transmission (seconds)
bool          firstTimeFlag     = true;   // Flag to determine if program is running for the first time
float         batteryCutoff     = 0.0;    // Battery voltage cutoff threshold (V)

// ------------------------------------------------------------------------------------------------
// Global variable declarations
// ------------------------------------------------------------------------------------------------
volatile bool alarmFlag         = false;  // Flag for alarm interrupt service routine
volatile bool wdtFlag           = false;  // Flag for Watchdog Timer interrupt service routine
volatile int  wdtCounter        = 0;      // Watchdog Timer interrupt counter
bool          resetFlag         = 0;      // Flag to force system reset using Watchdog Timer
uint8_t       moSbdBuffer[340];           // Buffer for Mobile Originated SBD (MO-SBD) message (340 bytes max)
uint8_t       mtSbdBuffer[270];           // Buffer for Mobile Terminated SBD (MT-SBD) message (270 bytes max)
size_t        moSbdBufferSize;
size_t        mtSbdBufferSize;
unsigned int  iterationCounter  = 0;      // Counter to track total number of program iterations (zero indicates a reset)
byte          samples           = 30;     // Number of samples to average accelerometer and magnetometer readings
byte          retransmitCounter = 0;      // Counter to track Iridium 9603 transmission reattempts
byte          transmitCounter   = 0;      // Counter to track Iridium 9603 transmission intervals
unsigned int  failureCounter    = 0;      // Counter to track consecutive failed Iridium transmission attempts
unsigned long previousMillis    = 0;      // Global millis() timer
unsigned long alarmTime         = 0;      // Global epoch alarm time variable
unsigned long unixtime          = 0;      // Global epoch time variable
unsigned int  sampleCounter     = 0;      // Global sensor measurement counter
unsigned int  cutoffCounter     = 0;      // Global battery voltage cutoff sleep cycle counter
float         temperatureInt    = 0.0;    // Internal temperature (°C)
float         humidityInt       = 0.0;    // Internal humidity (%)
float         pressureInt       = 0.0;    // Internal pressure (hPa)
float         latitude          = 0.0;    // GNSS latitude (DD)
float         longitude         = 0.0;    // GNSS longitude (DD)
byte          satellites        = 0;      // GNSS satellites
float         hdop              = 0.0;    // GNSS HDOP
float         voltage           = 0.0;    // Battery voltage
tmElements_t  tm;                         // Variable for converting time elements to time_t

// ------------------------------------------------------------------------------------------------
// Magnetometer min/max calibration
// ------------------------------------------------------------------------------------------------
// Based on:
// https://learn.adafruit.com/lsm303-accelerometer-slash-compass-breakout/calibration?view=all#calibration
// https://gist.github.com/CalebFenton/a97444750eb43e3354fd2d0196a2ebcf
// https://github.com/jremington/LSM9DS1-AHRS

float p[] = {1, 0, 0};  // X marking on sensor board points toward yaw = 0 (N)

// Min/max magnetometer values
float m_min[3] = {
  //0, 0, 0
  -76.35, -66.15, -23.40 // #1
  //-109.20, -71.40, -27.75 // #2
  //-107.25, -52.05, -94.95 // #3
  //-28.50, -39.30, -30.90 // #4
  //-72.30, -76.50, -96.75 // #5
  //-85.65, -62.25, -70.95 // #6
  //-62.55, -72.00, -51.75 // #7
  //-63.90, -81.00, -54.90 // #8
  //-97.05, -92.10, -112.80 // #9
  //-76.20, -80.40, -81.30 // #10
};

float m_max[3] = {
  //0, 0, 0
  51.90, 61.95, 105.60 // #1
  //21.15, 59.10, 102.45 // #2
  //25.50, 221.70, 132.75 // #3
  //93.45, 83.10, 95.85 // #4
  //51.00, 52.20, 36.45 // #5
  //41.10, 60.00, 56.85 // #6
  //58.35, 51.45, 72.75 // #7
  //51.45, 30.60, 72.15 // #8
  //26.70, 25.20, 11.25 // #9
  //41.55, 34.80, 41.85 // #10
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
    uint8_t   sampleInterval;     // 2 bytes
    uint8_t   averageInterval;    // 1 byte
    uint8_t   transmitInterval;   // 1 byte
    uint8_t   retransmitLimit;    // 1 byte
    uint8_t   batteryCutoff;      // 1 bytes
    uint8_t   resetFlag;          // 1 byte
  };
  uint8_t bytes[7]; // Size of message to be received in bytes
} SBD_MT_MESSAGE;

SBD_MT_MESSAGE mtSbdMessage;

// Structure to store device online/offline states
struct struct_online
{
  bool bme280   = false;
  bool lis2mdl  = false;
  bool lsm303   = false;
  bool gnss     = false;
} online;

// Union to store loop timers
struct struct_timer
{
  unsigned long readRtc;
  unsigned long readBattery;
  unsigned long readBme280;
  unsigned long readLsm303;
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
  pinMode(PIN_5V_EN, OUTPUT);
  pinMode(PIN_GNSS_EN, OUTPUT);
  pinMode(PIN_SENSOR_EN, OUTPUT);
  pinMode(PIN_IMU_EN, OUTPUT);
  pinMode(PIN_IRIDIUM_SLEEP, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(PIN_SENSOR_EN, LOW);   // Disable power to sensors
  digitalWrite(PIN_GNSS_EN, HIGH);    // Disable power to GNSS
  digitalWrite(PIN_IMU_EN, LOW);      // Disable power to IMU
  digitalWrite(PIN_5V_EN, LOW);       // Disable power to RockBLOCK 9603
  digitalWrite(PIN_IRIDIUM_SLEEP, HIGH);

  // Configure analog-to-digital (ADC) converter
  configureAdc();

  Wire.begin(); // Initialize I2C
  Wire.setClock(400000); // Set I2C clock speed to 400 kHz

#if DEBUG
  SERIAL_PORT.begin(115200); // Open serial port at 115200 baud
  blinkLed(LED_BUILTIN, 4, 500); // Non-blocking delay to allow user to open Serial Monitor
#endif

  DEBUG_PRINTLN();
  printLine();
  DEBUG_PRINT("Cryologger - Ice Tracking Beacon #"); DEBUG_PRINTLN(CRYOLOGGER_ID);
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

    // Increment the sample counter
    sampleCounter++;

    // Check if program is running for the first time
    if (!firstTimeFlag)
    {
      // Wake from deep sleep
      wakeUp();

      // Blink LED
      blinkLed(PIN_LED_RED, 4, 250);
    }

    DEBUG_PRINT("Info: Alarm trigger "); printDateTime();

    // Read battery voltage
    readBattery();

    // Check if battery voltage is above cutoff threshold
    if (voltage < batteryCutoff)
    {
      cutoffCounter++;

      // In the event that the battery voltage never recovers, force a reset of the
      // system after 1 week
      if (cutoffCounter > 168)
      {
        // Force WDT reset
        while (1);
      }

      DEBUG_PRINTLN("Warning: Battery voltage cutoff exceeded. Entering deep sleep...");

      // Reset sample counter
      sampleCounter = 0;

      // Clear statistics objects
      clearStats();

      // Go to sleep
      setCutoffAlarm();
    }
    else
    {
      DEBUG_PRINT("Info: Battery voltage good: "); DEBUG_PRINTLN(voltage);

      // Clear battery cutoff counter
      cutoffCounter = 0;

      // Perform measurements
      readBattery();        // Read the battery voltage
      readGnss();           // Read the GNSS
      enableSensorPower();  // Enable power to sensor(s)
      enableImuPower();     // Enable power to IMU
      readLsm303();         // Read the IMU
      readBme280();         // Read sensor(s)
      disableSensorPower(); // Disable 3.3V power to sensor(s)
      disableImuPower();    // Disable 3.3V power to IMU

      // Check if number of samples collected has been reached and calculate statistics (if enabled)
      if ((sampleCounter == averageInterval) || firstTimeFlag)
      {
        calculateStats(); // Calculate statistics of variables to be transmitted
        writeBuffer();    // Write data to transmit buffer

        // Check if data transmission interval has been reached
        if ((transmitCounter == transmitInterval) || firstTimeFlag)
        {
          transmitData();   // Transmit data via Iridium transceiver
        }
        sampleCounter = 0; // Reset sample counter
      }

      // Print function execution timers
      printTimers();

      // Set the RTC alarm
      setRtcAlarm();

      DEBUG_PRINTLN("Info: Entering deep sleep...");
      DEBUG_PRINTLN();

      // Prepare for sleep
      prepareForSleep();
    }
  }

  // Check for WDT interrupts
  if (wdtFlag)
  {
    petDog(); // Reset the WDT
  }

  // Blink LED to indicate WDT interrupt and nominal system operation
  blinkLed(LED_BUILTIN, 1, 25);

  // Enter deep sleep and wait for WDT or RTC alarm interrupt
  goToSleep();
}
