/*
    Title:    Cryologger Ice Tracking Beacon (ITB)
    Date:     June 22, 2019
    Author:   Adam Garbo

    Components:
    - Adafruit Feather M0 Proto
    - Adafruit DS3231 RTC Precision Featherwing
    - Adafruit Ultimate GPS Featherwing
    - Rock Seven RockBLOCK 9603
    - Pololu LSM303D 3D Compass and Accelerometer
    - Pololu 3.3V, 2.6A Step-Down Voltage Regulator D24V22F3

    Comments:
      This code is currently powering the two iceberg tracking beacons deployed 
      off the coast of Milne Ice Shelf, Ellesmere Island, Nunavut, summer 2019.
*/
#include <Arduino.h>            // https://github.com/arduino/ArduinoCore-samd (Required before wiring_private.h)
#include <ArduinoLowPower.h>    // https://github.com/arduino-libraries/ArduinoLowPower
#include <DS3232RTC.h>          // https://github.com/JChristensen/DS3232RTC
#include <IridiumSBD.h>         // https://github.com/mikalhart/IridiumSBD
#include <LSM303.h>             // https://github.com/pololu/lsm303-arduino
#include <TinyGPS++.h>          // https://github.com/mikalhart/TinyGPSPlus
#include <Wire.h>               // https://www.arduino.cc/en/Reference/Wire
#include <wiring_private.h>     // pinPeripheral() function required for creating a new Serial instance

// Defined constants
#define GPS_EN_PIN              A5
#define VBAT_PIN                A7
#define RTC_INT_PIN             5
#define ROCKBLOCK_EN_PIN        6
#define ROCKBLOCK_RX_PIN        10
#define ROCKBLOCK_TX_PIN        11
#define ROCKBLOCK_SLEEP_PIN     12

#define DEBUG true

// Create a new Serial instance. For more information see: https://www.arduino.cc/en/Tutorial/SamdSercom
Uart Serial2 (&sercom1, ROCKBLOCK_RX_PIN, ROCKBLOCK_TX_PIN, SERCOM_RX_PAD_2, UART_TX_PAD_0);  // Create the new UART instance assigning it to pin 10 and 11
#define IridiumSerial   Serial2
#define GpsSerial       Serial1

// Attach interrupt handler to SERCOM for new Serial instance
void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}

// Object instantiations
DS3232RTC       myRTC(false);     // Tell constructor not to initialize the I2C bus
IridiumSBD      modem(IridiumSerial, ROCKBLOCK_SLEEP_PIN);
LSM303          lsm303;
TinyGPSPlus     gps;
time_t          t, unixtime, alarmTime;
tmElements_t    tm;

// User defined global variable declarations
uint32_t        alarmInterval           = 3600;             // Sleep duration (in seconds) between data sample acquisitions (default = 5 minutes/300 seconds)
uint16_t        transmitInterval        = 1;              // Number of message to be included in each RockBLOCK transmission (340 byte limit)
uint16_t        maxRetransmitCounter    = 2;              // Maximum number of failed data transmissions to reattempt in a single message (340 byte limit)

// Global variable declarations
volatile bool   alarmIsrWasCalled       = false;          // RTC interrupt service routine (ISR) flag
volatile bool   sleeping                = false;          // Watchdog Timer Early Warning interrupt flag
uint8_t         samplesToAverage        = 30;             // Number of data values to average
uint8_t         transmitBuffer[340]     = {};             // RockBLOCK transmission buffer
uint16_t        retransmitCounter       = 0;              // RockBLOCK failed data transmission counter
uint16_t        messageCounter          = 0;              // RockBLOCK transmitted message counter
uint16_t        transmitCounter         = 0;              // RockBLOCK transmission interval counter
uint32_t        previousMillis          = 0;              // RockBLOCK callback function timer
uint32_t        transmitTime            = 0;              // RockBLOCK data transmission timer
uint8_t         forceReset              = 0;              // Watchdog Timer force reset flag
const float     alpha                   = 0.10;           // Alpha
float           fXa, fYa, fZa           = 0.0;            // Low-pass filtered accelerometer variables
extern "C" char *sbrk(int i);                             // Free RAM function

// Structure and union to store and send data byte-by-byte via RockBLOCK
typedef union
{
  struct
  {
    uint32_t    unixtime;           // Date and time in time_t format         (4 bytes)
    int16_t     temperature;        // DS3231SN temperature (°C)              (2 bytes)
    int16_t     voltage;            // Minimum battery voltage (mV)           (2 bytes)
    int16_t     pitch;              // LSM303D pitch (°)                      (2 bytes)
    int16_t     roll;               // LSM303D roll (°)                       (2 bytes)
    uint16_t    heading;            // LSM303D tilt-compensated heading (°)   (2 bytes)
    int32_t     latitude;           // GPS latitude (°)                       (4 bytes)
    int32_t     longitude;          // GPS longitude (°)                      (4 bytes)
    uint16_t    satellites;         // GPS number of satellites               (2 bytes)
    uint16_t    hdop;               // GPS HDOP                               (2 bytes)
    uint16_t    messageCounter;     // RockBLOCK data transmission counter    (2 bytes)
    uint16_t    transmitTime;       // Debugging variable                     (2 bytes)
  } __attribute__((packed));                                                // Total = 30 bytes
  uint8_t bytes[];
} SBDMESSAGE;

SBDMESSAGE message;
size_t messageSize = sizeof(message);     // Size (in bytes) of data to be stored and transmitted

void setup()
{
  // Pin assignments
  pinMode(GPS_EN_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ROCKBLOCK_EN_PIN, OUTPUT);
  digitalWrite(GPS_EN_PIN, HIGH);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(ROCKBLOCK_EN_PIN, LOW);

  Wire.begin();           // Initiate the Wire library
  Serial.begin(115200);
  //while (!Serial);      // Wait for monitoring Serial
  delay(10000);

  // Configure Watchdog Timer
  configureWatchdog();

  // Define device power profile assuming battery power
  modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE);
  modem.adjustSendReceiveTimeout(90);
  modem.adjustATTimeout(20);

  // Initialize DS3231
  myRTC.begin();                                // Initialize the I2C bus
  myRTC.setAlarm(ALM1_MATCH_DATE, 0, 0, 0, 1);  // Initialize the alarms to known values
  myRTC.setAlarm(ALM2_MATCH_DATE, 0, 0, 0, 1);
  myRTC.alarm(ALARM_1);                         // Clear the alarm flags
  myRTC.alarm(ALARM_2);
  myRTC.alarmInterrupt(ALARM_1, false);         // Clear the alarm interrupt flags
  myRTC.alarmInterrupt(ALARM_2, false);
  myRTC.squareWave(SQWAVE_NONE);

  // Configure an interrupt on the INT/SQW pin
  pinMode(RTC_INT_PIN, INPUT_PULLUP);
  LowPower.attachInterruptWakeup(RTC_INT_PIN, alarmIsr, FALLING);

  /*
    // Set the RTC time
    tm.Hour = 12;
    tm.Minute = 0;
    tm.Second =55;
    tm.Day = 30;
    tm.Month = 5;
    tm.Year = 2019 - 1970;    // tmElements_t.Year is the offset from 1970
    time_t t = makeTime(tm);  // change the tm structure into time_t (seconds since epoch)
    myRTC.set(t);
  */

  // Print current date and time
  printDatetime(myRTC.get());

  Serial.print(F("messageSize: ")); Serial.println(messageSize);

  // Set alarm 1
  //myRTC.setAlarm(ALM1_MATCH_SECONDS, 0, 0, 0, 1);     // Set initial alarm to occur at seconds rollover
  myRTC.setAlarm(ALM1_MATCH_MINUTES, 0, 0, 0, 1);     // Set initial alarm to occur at minutes rollover
  myRTC.alarm(ALARM_1);                               // Ensure alarm 1 interrupt flag is cleared
  myRTC.alarmInterrupt(ALARM_1, true);                // Enable interrupt output for alarm 1
}

void loop()
{
  if (alarmIsrWasCalled)
  {
    // Check to see if the alarm flag is set (also resets the flag if set)
    if (myRTC.alarm(ALARM_1))
    {
      resetWatchdog();    // Pet the Watchdog Timer
      myRTC.read(tm);     // Read current date and time
      t = makeTime(tm);   // Change the tm structure into time_t (seconds since epoch)
      printDatetime(t);   // Print ALARM_1 date and time

      // Perform measurements
      readDs3231();     // Read current date and time and internal temperature
      readBattery();    // Read battery voltage
      readLsm303();     // Read pitch, roll and tilt-compensated heading from LSM303 sensor (°)
      readGps();        // Read GPS coordinates
      writeBuffer();    // Write data to transmit buffer

      // Check if data is to be transmitted
      if (transmitCounter == transmitInterval)
      {
        transmitData();                   // Transmit data
        transmitCounter = 0;              // Reset transmit counter
      }

      // Set alarm 1
      alarmTime = t + alarmInterval;      // Calculate next alarm
      myRTC.setAlarm(ALM1_MATCH_DATE, second(alarmTime), minute(alarmTime), hour(alarmTime), day(alarmTime)); // Set alarm
      myRTC.alarm(ALARM_1);               // Ensure alarm 1 interrupt flag is cleared
      Serial.print(F("Next alarm: "));
      printDatetime(alarmTime);

      // Check if alarm 1 is set in the past
      if (myRTC.get() >= alarmTime)
      {
        t = myRTC.get();                  // Read current date and time
        alarmTime = t + alarmInterval;    // Calculate next alarm
        myRTC.setAlarm(ALM1_MATCH_DATE, 0, 0, hour(alarmTime), day(alarmTime)); // Set alarm
        myRTC.alarm(ALARM_1);             // Ensure alarm 1 interrupt flag is cleared
        Serial.print(F("Alarm set in the past! New alarm time: "));
        printDatetime(alarmTime);
      }
    }
    alarmIsrWasCalled = false;
  }
  sleeping = true;
  blink(LED_BUILTIN, 2, 100);
  LowPower.deepSleep();                   // Enter deep sleep
}

// RTC interrupt service routine (ISR)
void alarmIsr()
{
  alarmIsrWasCalled = true;
}

// Measure internal temperature from DS3231 RTC
void readDs3231()
{
  uint32_t loopStartTime = millis();

  t = myRTC.get();                                // Read current date and time
  float temperature = myRTC.temperature() / 4.0;  // Read internal temperature of DS3231 RTC

  // Write data to union
  message.unixtime = t;
  message.temperature = temperature * 100;

  uint32_t rtcLoopTime = millis() - loopStartTime;
  Serial.print("readDs3231() function execution: "); Serial.print(rtcLoopTime); Serial.println(F(" ms"));
}

// Measure battery voltage from 330/100 kOhm divider
void readBattery()
{
  uint32_t loopStartTime = millis();

  for (uint8_t i = 0; i < 5; i++)
  {
    analogRead(VBAT_PIN);
    delay(1);
  }
  float voltage = analogRead(VBAT_PIN);
  voltage *= ((330000.0 + 100000.0) / 100000.0); // Multiply back 100,000 / (330,000 + 100,000) kOhm =
  voltage *= 3.3;   // Multiply by 3.3V reference voltage
  voltage /= 1024;  // Convert to voltage

  // Write data to union
  message.voltage = voltage * 1000;

  uint32_t batteryLoopTime = millis() - loopStartTime;
  //Serial.print("voltage: "); Serial.println(voltage);
  //Serial.print("readBattery() function execution: "); Serial.print(batteryLoopTime); Serial.println(F(" ms"));
}

void writeBuffer()
{
  messageCounter++;
  message.messageCounter = messageCounter;
  transmitCounter++;

  // Concatenate current message with existing message(s) stored in transmit buffer
  memcpy(transmitBuffer + (sizeof(message) * (transmitCounter + (retransmitCounter * transmitInterval) - 1)), message.bytes, sizeof(message)); // Copy message to transmit buffer

  // Print union/structure
  printUnion();
  printUnionBinary();
  printTransmitBuffer();
}

// Read pitch, roll and tilt-compensated heading from LSM303 (°)
void readLsm303()
{
  if (lsm303.init())
  {
    lsm303.enableDefault();     // Turn on accelerometer and magnetometer
    /*
      Calibration values: the default values of +/-32767 for each axis lead to an assumed
      magnetometer bias of 0. Use the Pololu LSM303 library Calibrate example program to
      determine appropriate values for each LSM303 sensor.
    */
    // 2 = 1
    // 1 = 3
    // 3 = 2
    lsm303.m_min = (LSM303::vector<int16_t>)
    {
      //-32767, -32767, -32767      // Default
      //-3493,  -3422,  -3688       // Arctic Bay Cryologger #1
      //-3560,  -3260,  -3969     // Arctic Bay Cryologger #2
      -3701,  -3273,  -3438     // Arctic Bay Cryologger #3

    };
    lsm303.m_max = (LSM303::vector<int16_t>)
    {
      //+32767, +32767, +32767      // Default

      //+3090,  +3434,  +3665       // Arctic Bay Cryologger #1
      //+3019,  +3528,  +3358     // Arctic Bay Cryologger #2
      +3055,  +3296,  +3688     // Arctic Bay Cryologger #3
    };

    // Average accelerometer readings
    fXa, fYa, fZa = 0.0;
    // Apply low-pass filter to accelerometer data
    for (byte i = 0; i < 30; i++)   // 30 samples
    {
      lsm303.read();   // Read accelerometer and magnetometer data
      fXa = lsm303.a.x * alpha + (fXa * (1.0 - alpha));
      fYa = lsm303.a.y * alpha + (fYa * (1.0 - alpha));
      fZa = lsm303.a.z * alpha + (fZa * (1.0 - alpha));
      delay(5);
    }

    // Write registers to put accelerometer and magenteometer in power-down mode
    lsm303.writeAccReg(0x20, 0x07);
    lsm303.writeMagReg(0x26, 0x03);

    // Calculate orientation
    float pitch = (atan2(-fXa, sqrt((int32_t)fYa * fYa + (int32_t)fZa * fZa))) * 180 / PI;
    float roll = (atan2(fYa, fZa)) * 180 / PI;
    float heading = lsm303.heading((LSM303::vector<int>) {
      1, 0, 0   // PCB orientation
    });

    // Write orientation data to union
    message.pitch = pitch * 100;
    message.roll = roll * 100;
    message.heading = heading * 10;

    Serial.print(F("pitch: ")); Serial.println(pitch);
    Serial.print(F("roll: ")); Serial.println(roll);
    Serial.print(F("heading: ")); Serial.println(heading);
  }
  else
  {
    Serial.println(F("Error: Could not initialize LSM303!"));
  }
}

// Read latitude, longitude, number of satellites and HDOP from GPS
void readGps()
{
  unsigned long loopStartTime = millis();
  bool fixFound = false;
  bool charsSeen = false;

  // Enable GPS
  digitalWrite(GPS_EN_PIN, LOW);

  Serial.println("Beginning to listen for GPS traffic...");
  GpsSerial.begin(9600);
  delay(1000);

  // Configure GPS
  GpsSerial.println("$PMTK220,1000*1F");                                    // Set NMEA port update rate to 1Hz
  delay(100);
  GpsSerial.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");   // Set NMEA sentence output frequencies to GGA and RMC
  delay(100);
  //GpsSerial.println("$PGCMD,33,1*6C");                                      // Enable antenna updates
  GpsSerial.println("$PGCMD,33,0*6D");                                      // Disable antenna updates

  // Look for GPS signal for up to 2 minutes
  while (!fixFound && millis() - loopStartTime < 2UL * 60UL * 1000UL)
  {
    if (GpsSerial.available())
    {
      charsSeen = true;
      char c = GpsSerial.read();
      Serial.write(c);            // Echo NMEA sentences to serial
      if (gps.encode(c))
      {
        if ((gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) &&
            (gps.location.isUpdated() && gps.date.isUpdated() && gps.time.isUpdated()))
        {
          fixFound = true;
          message.latitude = gps.location.lat() * 1000000;
          message.longitude = gps.location.lng() * 1000000;
          message.satellites = gps.satellites.value();
          message.hdop = gps.hdop.value();

          // Sync real-time clock (RTC) with GPS time
          tm.Hour = gps.time.hour();
          tm.Minute = gps.time.minute();
          tm.Second = gps.time.second();
          tm.Day = gps.date.day();
          tm.Month = gps.date.month();
          tm.Year = gps.date.year() - 1970;   // tmElements_t.Year is the offset from 1970
          time_t gpsTime = makeTime(tm);      // Change the tm structure into time_t (seconds since epoch)

          // Compare current date and time of GPS and real-time clock
          Serial.println();
          Serial.print(F("RTC time: "));
          printDatetime(myRTC.get());
          Serial.print(F("GPS time: "));
          printDatetime(gpsTime);
          time_t drift = myRTC.get() - gpsTime;
          Serial.print(F("Drift: ")); Serial.print(drift); Serial.print(F(" seconds"));

          // Set real-time clock if drift exceeds 30 seconds
          if (drift > 15 || drift < -15)
          {
            myRTC.set(gpsTime);
            Serial.print(F("Real-time clock synced with GPS date and time"));
          }
        }
      }
    }

    ISBDCallback();

    if ((millis() - loopStartTime) > 5000 && gps.charsProcessed() < 10)
    {
      Serial.println(F("No GPS data received: check wiring"));
      break;
    }
  }
  Serial.println(charsSeen ? fixFound ? F("A GPS fix was found!") : F("No GPS fix was found.") : F("Wiring error: No GPS data seen."));

  uint32_t gpsLoopTime = millis() - loopStartTime;
  Serial.print("readGps() function execution: "); Serial.print(gpsLoopTime); Serial.println(F(" ms"));

  // Disable GPS
  digitalWrite(GPS_EN_PIN, HIGH);
}

// Transmit data via the RockBLOCK 9603 satellite modem
void transmitData()
{
  int signalQuality = -1;
  int16_t err;
  uint32_t loopStartTime = millis();

  digitalWrite(ROCKBLOCK_EN_PIN, HIGH);

  // Start the serial power connected to the RockBLOCK modem
  IridiumSerial.begin(19200);

  // Assign pins for SERCOM functionality for new Serial2 instance
  pinPeripheral(ROCKBLOCK_TX_PIN, PIO_SERCOM);
  pinPeripheral(ROCKBLOCK_RX_PIN, PIO_SERCOM);

  // Start the RockBLOCK modem
  err = modem.begin();
  if (err == ISBD_SUCCESS)
  {
    uint8_t inBuffer[9];   // Buffer to store incomming RockBLOCK transmission (340 byte limit per message)
    size_t inBufferSize = sizeof(inBuffer);
    memset(inBuffer, 0x00, sizeof(inBuffer));   // Clear  inBuffer array

#if DEBUG
    // Test the signal quality
    err = modem.getSignalQuality(signalQuality);
    if (err != ISBD_SUCCESS)
    {
      Serial.print("SignalQuality failed: error ");
      Serial.println(err);
    }
    Serial.print("Signal quality: "); Serial.println(signalQuality);
#endif

    // Transmit and receieve data in binary format
    err = modem.sendReceiveSBDBinary(transmitBuffer, (sizeof(message) * (transmitCounter + (retransmitCounter * transmitInterval))), inBuffer, inBufferSize);

    if (err == ISBD_SUCCESS) {
      retransmitCounter = 0;
      memset(transmitBuffer, 0x00, sizeof(transmitBuffer));   // Clear transmit buffer array

      // Check for incoming message. If no inbound message is available, set inBufferSize = 0
      if (inBufferSize > 0)
      {
        // Print inBuffer size and values of each incoming byte of data
        Serial.print(F("Inbound buffer size is: ")); Serial.println(inBufferSize);
        for (uint8_t i = 0; i < inBufferSize; i++)
        {
          Serial.print(F("address: ")); Serial.print(i); Serial.print(F("\tvalue: ")); Serial.println(inBuffer[i], HEX);
        }

        // Recompose variables from inBuffer using bitshift
        uint8_t forceResetBuffer            = (((uint8_t)inBuffer[8] << 0) & 0xFF);
        uint16_t maxRetransmitCounterBuffer = (((uint16_t)inBuffer[7] << 0) & 0xFF) +
                                              (((uint16_t)inBuffer[6] << 8) & 0xFFFF);
        uint16_t transmitIntervalBuffer     = (((uint16_t)inBuffer[5] << 0) & 0xFF) +
                                              (((uint16_t)inBuffer[4] << 8) & 0xFFFF);
        uint32_t alarmIntervalBuffer        = (((uint32_t)inBuffer[3] << 0) & 0xFF) +
                                              (((uint32_t)inBuffer[2] << 8) & 0xFFFF) +
                                              (((uint32_t)inBuffer[1] << 16) & 0xFFFFFF) +
                                              (((uint32_t)inBuffer[0] << 24) & 0xFFFFFFFF);

        // Check if inBuffer data is valid
        if ((alarmIntervalBuffer > 300  && alarmIntervalBuffer <= 2678400) &&
            (transmitIntervalBuffer > 0  && transmitIntervalBuffer <= 11) &&
            (maxRetransmitCounterBuffer > 0  && maxRetransmitCounterBuffer <= 10) &&
            (forceResetBuffer == 0 || forceResetBuffer == 255))
        {
          alarmInterval = alarmIntervalBuffer;
          transmitInterval = transmitIntervalBuffer;
          maxRetransmitCounter = maxRetransmitCounterBuffer;
          forceReset = forceResetBuffer;
        }
      }
    }
    else
    {
      Serial.print(F("Transmission failed: error "));
      Serial.println(err);
    }
  }
  else
  {
    Serial.print("Begin failed: error ");
    Serial.println(err);
  }

  // If transmission or modem begin fail, stored message in transmit buffer
  if (err != ISBD_SUCCESS)
  {
    retransmitCounter++;  // Increment retransmit counter

    // Reset retransmit counter if maximum messages reattempts is exceeded
    if (retransmitCounter >= maxRetransmitCounter)
    {
      retransmitCounter = 0;
      memset(transmitBuffer, 0x00, sizeof(transmitBuffer));   // Clear transmitBuffer array
    }
  }

  // Put the RockBLOCK into low power "sleep" mode
  err = modem.sleep();
  if (err != ISBD_SUCCESS)
  {
    Serial.print(F("Sleep failed: error "));
    Serial.println(err);
  }

  // Close the serial port connected to the RockBLOCK modem
  IridiumSerial.end();

  // Disable power to RockBLOCK
  digitalWrite(ROCKBLOCK_EN_PIN, LOW);

  transmitTime = millis() - loopStartTime;
  message.transmitTime = transmitTime / 1000;
  uint32_t iridiumLoopTime = millis() - loopStartTime;

  Serial.print("transmitData() function execution: "); Serial.print(iridiumLoopTime); Serial.println(F(" ms"));
  Serial.print(F("transmitTime: ")); Serial.println(transmitTime / 1000);
  Serial.print(F("retransmitCounter: ")); Serial.println(retransmitCounter);
  Serial.print(F("alarmInterval: ")); Serial.println(alarmInterval);
  Serial.print(F("transmitInterval: ")); Serial.println(transmitInterval);
  Serial.print(F("maxRetransmitCounter: ")); Serial.println(maxRetransmitCounter);
  Serial.print(F("forceReset: ")); Serial.println(forceReset);

  // Force Watchdog Timer to reset the system
  if (forceReset == 255)
  {
    digitalWrite(13, HIGH);
    while (true);
  }
}

// RockBLOCK callback function
bool ISBDCallback()   // This function can be repeatedly called during data transmission or GPS signal acquisition
{
#if DEBUG
  digitalWrite(LED_BUILTIN, (millis() / 1000) % 2 == 1 ? HIGH : LOW);
#endif
  uint32_t currentMillis = millis();
  if (currentMillis - previousMillis >= 1000)
  {
    previousMillis = currentMillis;
    readBattery();    // Measure battery voltage
    resetWatchdog();  // Pet the Watchdog Timer
  }
  return true;
}

// Callback to sniff the conversation with the Iridium modem
void ISBDConsoleCallback(IridiumSBD *device, char c)
{
#if DEBUG
  Serial.write(c);
#endif
}

// Callback to to monitor library's run state
void ISBDDiagsCallback(IridiumSBD *device, char c)
{
#if DEBUG
  Serial.write(c);
#endif
}

// Print union/structure
void printUnion()
{
  Serial.println();
  Serial.println(F("Union/structure"));
  Serial.println(F("==================================="));
  Serial.print(F("unixtime:\t\t")); Serial.println(message.unixtime);
  Serial.print(F("temperature:\t\t")); Serial.println(message.temperature);
  Serial.print(F("pitch:\t\t\t")); Serial.println(message.pitch);
  Serial.print(F("roll:\t\t\t")); Serial.println(message.roll);
  Serial.print(F("heading:\t\t")); Serial.println(message.heading);
  Serial.print(F("latitude:\t\t")); Serial.println(message.latitude);
  Serial.print(F("longitude:\t\t")); Serial.println(message.longitude);
  Serial.print(F("satellites:\t\t")); Serial.println(message.satellites);
  Serial.print(F("hdop:\t\t\t")); Serial.println(message.hdop);
  Serial.print(F("voltage:\t\t")); Serial.println(message.voltage);
  Serial.print(F("messageCounter:\t\t")); Serial.println(message.messageCounter);
  Serial.print(F("transmitTime:\t\t")); Serial.println(message.transmitTime);
}

// Print contents of union/structure
void printUnionBinary()
{
  Serial.println();
  Serial.println(F("Union/structure"));
  Serial.println(F("========================="));
  Serial.println(F("Byte\tHex\tBinary"));
  for (uint16_t i = 0; i < sizeof(message); ++i)
  {
    Serial.print(i);
    Serial.print("\t");
    Serial.print(message.bytes[i], HEX);
    Serial.print("\t");
    Serial.println(message.bytes[i], BIN);
  }
}

// Print contents of transmiff buffer array
void printTransmitBuffer()
{
  Serial.println();
  Serial.println(F("Transmit buffer"));
  Serial.println(F("========================="));
  Serial.println(F("Byte\tHex\tBinary"));
  for (uint16_t i = 0; i < 340; i++)
  {
    Serial.print(i);
    Serial.print("\t");
    Serial.print(transmitBuffer[i], HEX);
    Serial.print("\t");
    Serial.println(transmitBuffer[i], BIN);
  }
}


// Print current time and date
void printDatetime(time_t t)
{
  Serial.print((day(t) < 10) ? "0" : ""); Serial.print(day(t), DEC); Serial.print('/');
  Serial.print((month(t) < 10) ? "0" : ""); Serial.print(month(t), DEC); Serial.print('/');
  Serial.print(year(t), DEC); Serial.print(' ');
  Serial.print((hour(t) < 10) ? "0" : ""); Serial.print(hour(t), DEC); Serial.print(':');
  Serial.print((minute(t) < 10) ? "0" : ""); Serial.print(minute(t), DEC); Serial.print(':');
  Serial.print((second(t) < 10) ? "0" : ""); Serial.println(second(t), DEC);
}

// LED activity indicator
void blink(uint8_t pin, uint16_t flashes, uint16_t duration)
{
  for (uint16_t i = 0; i < flashes; i++)
  {
    digitalWrite(pin, HIGH);
    delay(duration);
    digitalWrite(pin, LOW);
    delay(duration);
  }
}

int freeRam()
{
  char stack_dummy = 0;
  return &stack_dummy - sbrk(0);
}

// Set up the WDT to perform a system reset if the loop() blocks for more than 8-16 seconds
void configureWatchdog()
{
  // Set up the generic clock (GCLK2) used to clock the watchdog timer at 1.024kHz
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(4) |            // Divide the 32.768kHz clock source by divisor 32, where 2^(4 + 1): 32.768kHz/32=1.024kHz
                    GCLK_GENDIV_ID(2);              // Select Generic Clock (GCLK) 2
  while (GCLK->STATUS.bit.SYNCBUSY);                // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_DIVSEL |          // Set to divide by 2^(GCLK_GENDIV_DIV(4) + 1)
                     GCLK_GENCTRL_IDC |             // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |           // Enable GCLK2
                     GCLK_GENCTRL_SRC_OSCULP32K |   // Set the clock source to the ultra low power oscillator (OSCULP32K)
                     GCLK_GENCTRL_ID(2);            // Select GCLK2
  while (GCLK->STATUS.bit.SYNCBUSY);                // Wait for synchronization

  // Feed GCLK2 to WDT (Watchdog Timer)
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |           // Enable GCLK2 to the WDT
                     GCLK_CLKCTRL_GEN_GCLK2 |       // Select GCLK2
                     GCLK_CLKCTRL_ID_WDT;           // Feed the GCLK2 to the WDT
  while (GCLK->STATUS.bit.SYNCBUSY);                // Wait for synchronization

  WDT->EWCTRL.bit.EWOFFSET = 0xA;                   // Set the Early Warning Interrupt Time Offset to 8 seconds //REG_WDT_EWCTRL = WDT_EWCTRL_EWOFFSET_8K;
  WDT->INTENSET.bit.EW = 1;                         // Enable the Early Warning interrupt //REG_WDT_INTENSET = WDT_INTENSET_EW;
  WDT->CONFIG.bit.PER = 0xB;                        // Set the WDT reset timeout to 16 seconds //REG_WDT_CONFIG = WDT_CONFIG_PER_16K;
  WDT->CTRL.bit.ENABLE = 1;                         // Enable the WDT in normal mode //REG_WDT_CTRL = WDT_CTRL_ENABLE;
  while (WDT->STATUS.bit.SYNCBUSY);                 // Await synchronization of registers between clock domains

  // Configure and enable WDT interrupt
  NVIC_DisableIRQ(WDT_IRQn);
  NVIC_ClearPendingIRQ(WDT_IRQn);
  NVIC_SetPriority(WDT_IRQn, 0);                    // Top priority
  NVIC_EnableIRQ(WDT_IRQn);
}

// Pet the Watchdog Timer
void resetWatchdog()
{
  WDT->CLEAR.bit.CLEAR = 0xA5;                      // Clear the Watchdog Timer and restart time-out period //REG_WDT_CLEAR = WDT_CLEAR_CLEAR_KEY;
  while (WDT->STATUS.bit.SYNCBUSY);                 // Await synchronization of registers between clock domains
}

// Watchdog Timer interrupt service routine (ISR)
void WDT_Handler()
{
  if (sleeping)
  {
    sleeping = false;
    WDT->INTFLAG.bit.EW = 1;                        // Clear the Early Warning interrupt flag //REG_WDT_INTFLAG = WDT_INTFLAG_EW;
    WDT->CLEAR.bit.CLEAR = 0xA5;                    // Clear the Watchdog Timer and restart time-out period //REG_WDT_CLEAR = WDT_CLEAR_CLEAR_KEY;
    while (WDT->STATUS.bit.SYNCBUSY);               // Await synchronization of registers between clock domains
  }
  else
  {
    //WDT->CTRL.bit.ENABLE = 0;                       // DEBUG - Disable Watchdog
    //digitalWrite(13, HIGH);                         // DEBUG - Turn on LED to indicate Watchdog trigger
    while (true);                                   // Force Watchdog Timer to reset the system
  }
}
