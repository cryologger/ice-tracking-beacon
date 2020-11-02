/*
    Title:    Cryologger Ice Tracking Beacon (ITB) - Version 3
    Date:     October 31, 2020
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

#include <IridiumSBD.h>                     // http://librarymanager/All#IridiumSBDI2C
#include <RTC.h>
#include <SparkFun_Ublox_Arduino_Library.h> // http://librarymanager/All#SparkFun_Ublox_GPS
#include <SdFat.h>                          // http://librarymanager/All#SdFat
#include <SPI.h>
#include <Wire.h>

// Defined constants
#define DEBUG         true    // Output debug messages to Serial Monitor
#define DIAGNOSTICS   false    // Output Iridium diagnostic messages to Serial Monitor

#define IridiumWire Wire

// Pin definitions
#define PIN_PWC_POWER           G1
#define PIN_QWIIC_POWER         G2
#define PIN_MICROSD_CHIP_SELECT 41

// Object instantiations
APM3_RTC      rtc;
IridiumSBD    modem(IridiumWire);   // I2C Address: 0x63
SdFat         sd;                   // File system object
SdFile        file;                 // Log file
SFE_UBLOX_GPS gps;                  // I2C Address: 0x42

// User defined global variable declarations
byte          alarmSeconds          = 0;
byte          alarmMinutes          = 5;
byte          alarmHours            = 0;
unsigned int  transmitInterval      = 1;       // Number of message to be included in each RockBLOCK transmission (340 byte limit)
unsigned int  maxRetransmitCounter  = 1;        // Maximum number of failed data transmissions to reattempt in a single message (340 byte limit)

// Global variable and constant declarations
volatile bool alarmFlag           = false;   // RTC alarm flag
bool          ledState            = LOW;    // LED flag for blinkLed()
int           valFix              = 0;
int           maxValFix           = 10;

char          transmitBuffer[340] = {};   // Qwiic Iridium 9603N transmission buffer
unsigned int  messageCounter      = 0;    // Qwiic Iridium 9603N transmitted message counter
unsigned int  retransmitCounter   = 0;    // Qwiic Iridium 9603N failed data transmission counter
unsigned int  transmitCounter     = 0;    // Qwiic Iridium 9603N transmission interval counter
unsigned long previousMillis      = 0;

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
  uint8_t bytes[19];
} SBDMESSAGE;

SBDMESSAGE message;
size_t messageSize = sizeof(message);   // Size (in bytes) of data message to be transmitted

void setup() {

  // Pin assignments
  pinMode(PIN_QWIIC_POWER, OUTPUT);
  pinMode(PIN_PWC_POWER, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  qwiicPowerOn(); // Enable power to Qwiic connector
  peripheralPowerOn(); // Enable power to peripherials

  Wire.begin(); // Initialize I2C
  SPI.begin();  // Initialize SPI

  Serial.begin(115200);
  while (!Serial); // Wait for user to open Serial Monitor
  //delay(10000); // Delay to allow user to open Serial Monitor

  Serial.println(F("-----------------------------------------"));
  Serial.println(F("Cryologger - Iceberg Tracking Beacon v3.0"));
  Serial.println(F("-----------------------------------------"));

  Serial.print(F("Datetime: "));
  printDateTime();
  Serial.print(F("UNIX Epoch time: "));
  Serial.println(rtc.getEpoch());

  configureSd();      // Configure microSD
  configureGnss();    // Configure u-blox SAM-M8Q receiver
  configureIridium(); // Configure Qwiic Iridium 9603N
  configureRtc();     // Configure real-time clock (RTC)

  Serial.flush(); // Wait for transmission of serial data to complete
}

void loop() {

  // Check if alarm flag was set
  if (alarmFlag) {

    alarmFlag = false; // Clear alarm flag

    // Print date and time of RTC alarm trigger
    Serial.print("Alarm trigger: ");
    printDateTime();

    
    readGnss(); // Read GNSS
    transmitData(); // Transmit data

    // Set the RTC's rolling alarm
    rtc.setAlarm((rtc.hour + alarmHours) % 24,
                 (rtc.minute + alarmMinutes) % 60,
                 (rtc.seconds + alarmSeconds) % 60,
                 0, rtc.dayOfMonth, rtc.month);
    rtc.setAlarmMode(4);

    // Print next RTC alarm date and time
    Serial.print("Next alarm: ");
    printAlarm();
  }

  blinkLed(1, 100);

  // Enter deep sleep and await RTC alarm interrupt
  //goToSleep();
}


// Blink LED (non-blocking)
void blinkLed(byte flashes, unsigned int interval) {

  pinMode(LED_BUILTIN, OUTPUT);
  byte i = 0;

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

// Interrupt handler for the RTC
extern "C" void am_rtc_isr(void)
{
  // Clear the RTC alarm interrupt
  am_hal_rtc_int_clear(AM_HAL_RTC_INT_ALM);

  // Set alarm flag
  alarmFlag = true;
}
