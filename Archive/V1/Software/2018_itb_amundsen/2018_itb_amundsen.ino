/*
  Title:          Cryologger Ice Tracking Beacon (ITB)
  Author:         Adam Garbo
  Last modified:  August 25, 2018
  Description:    Ice tracking beacon intended for deployment in the Canadian Arctic.
*/

// Libraries
#include <avr/interrupt.h>      // https://www.nongnu.org/avr-libc/user-manual/group__avr__interrupts.html
#include <avr/power.h>          // https://www.nongnu.org/avr-libc/user-manual/group__avr__power.html
#include <avr/sleep.h>          // https://www.nongnu.org/avr-libc/user-manual/group__avr__sleep.html
#include <avr/wdt.h>            // https://www.nongnu.org/avr-libc/user-manual/group__avr__watchdog.html
#include <DS3232RTC.h>          // https://github.com/JChristensen/DS3232RTC
#include <extEEPROM.h>          // https://github.com/JChristensen/extEEPROM
#include <IridiumSBD.h>         // https://github.com/mikalhart/IridiumSBD
#include <LSM303.h>             // https://github.com/pololu/lsm303-arduino
#include <SoftwareSerial.h>     // https://www.arduino.cc/en/Reference/SoftwareSerial (included with Arduino IDE)
#include <SparkFunMPL3115A2.h>  // https://github.com/sparkfun/MPL3115A2_Breakout/
#include <TinyGPS.h>            // https://github.com/mikalhart/TinyGPS
#include <Wire.h>               // https://www.arduino.cc/en/Reference/Wire (included with Arduino IDE)

// Defined constants
#define DEBUG                   // Echo data to serial port. Comment line to prevent all Serial.print() commands
#define LED               13    // LED activity and error indicator
#define RTC_ALARM_PIN     3     // Connect DS3231 SQW/INT to Pro Trinket D3 pin (INT1)
#define GPS_RX            4     // Connect GPS Rx to Pro Trinket D4 pin
#define GPS_TX            5     // Connect GPS Tx to Pro Trinket D5 pin
#define GPS_EN            6     // Connect GPS EN to Pro Trinket D6 pin
#define ROCKBLOCK_EN      8     // Connect Pololu step-down voltage regulator EN to Pro Trinket D8 pin
#define ROCKBLOCK_SLEEP   9     // Connect RockBLOCK Sleep to Pro Trinket D9 pin
#define ROCKBLOCK_TX      10    // Connect RockBLOCK Tx to Pro Trinket D10 pin
#define ROCKBLOCK_RX      11    // Connect RockBLOCK Rx to Pro Trinket D11 pin

// Software serial ports
SoftwareSerial    ssGps(GPS_TX, GPS_RX);
SoftwareSerial    ssIridium(ROCKBLOCK_RX, ROCKBLOCK_TX);

// Object instantiations
extEEPROM       extEeprom(kbits_512, 1, 128, 0x50);
IridiumSBD      isbd(ssIridium, ROCKBLOCK_SLEEP);
LSM303          lsm303;
MPL3115A2       mpl3115a2;
time_t          t, alarmTime;
tmElements_t    tm;
TinyGPS         tinyGps;

// User defined global variable declarations
unsigned long   alarmInterval           = 3600;    // Sleep duration (in seconds) between data sample acquisitions
unsigned int    transmitInterval        = 1;      // Number of data samples to be included in a single transmission (340 byte limit)
unsigned int    maxRetransmitCounter    = 9;      // Maximum number of failed data transmissions to reattempt in a single message (340 byte limit)

// Global variable declarations
const unsigned int    maxEepromAddress        = 30000;    // Maximum writeable address of external EEPROM
volatile bool         alarmIsrWasCalled       = false;    // DS3231 RTC alarm interrupt service routine (ISR) flag
volatile bool         wdtIsrWasCalled         = true;     // Watchdog Timer (WDR) ISR flag
volatile bool         sleeping                = true;     // Sleep flag for WDT ISR
unsigned int          retransmitCounter       = 0;        // RockBLOCK failed data transmission counter
unsigned int          currentEepromAddress    = 0;        // Initial external EEPROM write location
unsigned int          transmitCounter         = 0;        // RockBLOCK data transmission counter
unsigned int          messageCounter          = 0;        // Global RockBLOCK data transmission counter
unsigned int          transmitDuration        = 0;        // Global RockBLOCK data transmission time variable
unsigned long         previousMillis          = 0;        // Global RockBLOCK callback function timer variable

// Union
typedef union
{
  struct
  {
    unsigned long   unixtime;           // (4 bytes)  Date and time in time_t format (seconds since epoch)
    //int             tempDs3231;         // (2 bytes)  DS3231 RTC internal temperature (°C)
    int             temperature;        // (2 bytes)  MPL3115A2 temperature (°C)
    unsigned int    pressure;           // (2 bytes)  MPL3115A2 pressure (Pa)
    int             pitch;              // (2 bytes)  LSM303 pitch (°)
    int             roll;               // (2 bytes)  LSM303 roll (°)
    unsigned int    heading;            // (2 bytes)  LSM303 tilt-compensated heading (°)
    long            latitude;           // (4 bytes)  GPS latitude (DD)
    long            longitude;          // (4 bytes)  GPS longitude (DD)
    unsigned int    satellites;         // (2 bytes)  GPS number of satellites
    unsigned int    hdop;               // (2 bytes)  GPS HDOP
    unsigned int    voltage;            // (2 bytes)  Battery voltage (mV)
    unsigned int    transmitDuration;   // (2 bytes)  Data transmission time (seconds)
    unsigned int    messageCounter;   // (2 bytes)  Data transmission counter

  } __attribute__((packed));            // 30 bytes total
  byte bytes[];
} SBDMessage;

SBDMessage message;
size_t messageSize = sizeof(message); // Size (in bytes) of data to be stored and transmitted

// Setup
void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
#endif // DEBUG

  Wire.begin();   // Enable SDA & SCL internal pull-up resistors

  // Configure digital pins
  pinMode(LED, OUTPUT);               // LED pin
  pinMode(GPS_EN, OUTPUT);            // GPS EN (enable) pin
  pinMode(ROCKBLOCK_EN, OUTPUT);      // Step-down voltage regulator EN (enable) pin
  digitalWrite(GPS_EN, LOW);          // Set GPS EN pin LOW (0 V) to disable onboard regulator
  digitalWrite(ROCKBLOCK_EN, LOW);    // Set step-down EN pin LOW (0 V) to disable step-down voltage regulator

  // Configure the Watchdog Timer
  configureWatchdog();

  // External EEPROM
  byte i2cStat = extEeprom.begin(extEEPROM::twiClock400kHz);
  if (i2cStat != 0)
  {
#ifdef DEBUG
    Serial.println(F("Unable to initialize EEPROM!"));
#endif // DEBUG
  }

  // Configure RockBLOCK
  isbd.attachConsole(Serial);       // Monitor serial traffic to/from the RockBLOCK
  isbd.attachDiags(Serial);         // Diagnostic routine to monitor debug messages
  isbd.setPowerProfile(0);          // Defines the internal delays between retransmission (0 = direct connect (default), 1 = USB)
  isbd.useMSSTMWorkaround(false);   // Disable “MSSTM” software query workaround

  // DS3231 Real-time clock (RTC)
  RTC.setAlarm(ALM1_MATCH_DATE, 0, 0, 0, 1);    // Initialize alarm 1 to known value
  RTC.setAlarm(ALM2_MATCH_DATE, 0, 0, 0, 1);    // Initialize alarm 2 to known value
  RTC.alarm(ALARM_1);                           // Clear alarm 1 interrupt flag
  RTC.alarm(ALARM_2);                           // Clear alarm 2 interrupt flag
  RTC.alarmInterrupt(ALARM_1, false);           // Disable interrupt output for alarm 1
  RTC.alarmInterrupt(ALARM_2, false);           // Disable interrupt output for alarm 2
  RTC.squareWave(SQWAVE_NONE);                  // Configure INT/SQW pin for interrupt operation by disabling default square wave output

  /*
    // Set RTC to an arbitrary time (for debugging purposes)
    tm.Hour = 10;
    tm.Minute = 59;
    tm.Second = 55;
    tm.Day = 26;
    tm.Month = 7;
    tm.Year = 2018 - 1970;    // tmElements_t.Year is the offset from 1970
    time_t t = makeTime(tm);  // change the tm structure into time_t (seconds since epoch)
    RTC.set(t);
  */

#ifdef DEBUG
  Serial.print(F("Cryologger #6 "));
  printDateTime(RTC.get());   // Display current date and time
#endif // DEBUG

  pinMode(RTC_ALARM_PIN, INPUT_PULLUP);       // Enable internal pull-up resistor on Pro Trinket external interrupt pin D3
  attachInterrupt(INT1, alarmIsr, FALLING);   // Wake on falling edge of RTC_ALARM_PIN

  // Set alarm 1
  //RTC.setAlarm(ALM1_MATCH_SECONDS, 0, 0, 0, 1);     // Set initial alarm to occur at seconds rollover
  RTC.setAlarm(ALM1_MATCH_MINUTES, 0, 0, 0, 1);     // Set initial alarm to occur at minutes rollover
  RTC.alarm(ALARM_1);                               // Ensure alarm 1 interrupt flag is cleared
  RTC.alarmInterrupt(ALARM_1, true);                // Enable interrupt output for alarm 1
}

// Loop
void loop()
{
  if ((alarmIsrWasCalled) || (wdtIsrWasCalled))
  {
    t = RTC.get();            // Read current date and time from RTC
    if (RTC.alarm(ALARM_1))   // Check alarm 1 and clear flag if set
    {
#ifdef DEBUG
      Serial.println(F("===================================="));
      Serial.print(F("Cryologger #6 Alarm: ")); printDateTime(t);
      Serial.println(F("===================================="));
#endif // DEBUG

      transmitCounter++;    // Increment data transmission interval counter
      messageCounter++;     // Increment data transmission counter (provides indication of system reset)
      message.messageCounter = messageCounter;  // Write iteration counter value to union

#ifdef DEBUG
      Serial.print(F("transmitCounter: ")); Serial.println(transmitCounter);
      Serial.print(F("messageCounter: ")); Serial.println(messageCounter);
#endif // DEBUG

      // Read sensors
      readBattery();    // Read battery voltage (mV) from voltage divider
      readDs3231();     // Read date and time from RTC
      readMpl3115a2();  // Read temperature (°C) and pressure (Pa) from MPL3115A2 sensor
      readGps();        // Read latitude and longitude (DD), number of satellites, and HDOP from GPS
      readLsm303();     // Read pitch, roll and tilt-compensated heading from LSM303 sensor (°)
      writeEeprom();    // Write data stored in union to external EEPROM

      // Data transmission check
      if (transmitCounter == transmitInterval)
      {
        transmitData();
        transmitCounter = 0;
      }
#ifdef DEBUG
      Serial.print(F("transmitCounter: ")); Serial.println(transmitCounter);
#endif // DEBUG

      // Set alarm 1
      alarmTime = t + alarmInterval;    // Calculate next alarm
      RTC.setAlarm(ALM1_MATCH_DATE, second(alarmTime), minute(alarmTime), hour(alarmTime), day(alarmTime));   // Set alarm
      RTC.alarm(ALARM_1);               // Ensure alarm 1 interrupt flag is cleared

      // Check if alarm was set in the past
      if (RTC.get() >= alarmTime)
      {
#ifdef DEBUG
        Serial.println(F("*** Warning! Alarm set in the past ***"));
        Serial.print(F("Previous alarm: ")); printDateTime(alarmTime);
#endif // DEBUG
        t = RTC.get();                    // Read current date and time from RTC
        alarmTime = t + alarmInterval;    // Calculate next alarm
        RTC.setAlarm(ALM1_MATCH_DATE, second(alarmTime), minute(alarmTime), hour(alarmTime), day(alarmTime));
        RTC.alarm(ALARM_1);               // Ensure alarm 1 interrupt flag is cleared
      }
#ifdef DEBUG
      Serial.print(F("Current time: ")); printDateTime(RTC.get());
      Serial.print(F("New alarm: ")); printDateTime(alarmTime);
#endif // DEBUG
    }
    blink(1, 5);                // Watchdog Timer visual indicator
    alarmIsrWasCalled = false;  // Reset RTC ISR flag
    wdtIsrWasCalled = false;    // Reset WDT ISR flag
    goToSleep();                // Sleep
  }
}

// Functions

// Measure battery voltage from 1 MOhm / 10 MOhm resistor voltage divider (Measured values: 10.05 MOhm / 992 kOhm)
void readBattery()
{
  for (int i = 0; i < 5; i++)   // Allow capacitor to settle before recording analog measurement
  {
    analogRead(A0);
    delay(1);
  }
  float voltage = ((float)analogRead(A0) + 0.5) / 93.1 * 3.3;   // Voltage measured at A0 is a fraction of the 1024 range: (1 MOhm / (1 MOhm + 10 MOhm) * 1024 = 91.99

  if (message.voltage == 0)                   // Check if battery voltage has been written to union
    message.voltage = voltage * 1000;         // Write initial battery voltage to union

  if ((voltage * 1000) <= message.voltage)    // Track minimum battery voltage
    message.voltage = voltage * 1000;         // Write battery voltage to union

#ifdef DEBUG
  //Serial.print(F("voltage: ")); Serial.println(voltage * 1000);
#endif // DEBUG
}

// Read current date, time and internal temperature from DS3231 RTC
void readDs3231()
{
  int tempDs3231 = RTC.temperature();   // Read raw internal temperature (10-bit/0.25°C resolution)
  message.unixtime = t;                 // Write current date and time to union
  //message.tempDs3231 = tempDs3231;      // Write DS3231 internal temperature to union
#ifdef DEBUG
  Serial.print(F("unixtime: ")); Serial.println(t);
  //Serial.print(F("tempDS3231: ")); Serial.println(tempDs3231);
#endif // DEBUG
}

// Real-time clock alarm interrupt service routine (ISR)
void alarmIsr()
{
  alarmIsrWasCalled = true;
}

// Read temperature (°C), humidity (%) and pressure (kPa) data from BME280 sensor
void readMpl3115a2()
{
  // Configure the sensor
  mpl3115a2.begin(); // Get sensor online
  mpl3115a2.setModeBarometer();     // Measure pressure in Pascals from 20 to 110 kPa
  mpl3115a2.setOversampleRate(7);   // Set Oversample to the recommended 128
  mpl3115a2.enableEventFlags();     // Enable all three pressure and temp event flags

  // Read sensor
  mpl3115a2.setModeActive();
  float temperature = mpl3115a2.readTemp();           // Read temperature (°C)
  float pressure = mpl3115a2.readPressure() / 1000;   // Read pressure (kPa)
  mpl3115a2.setModeStandby();

  // Write data to union
  message.temperature = temperature * 100;
  message.pressure = pressure * 100;

#ifdef DEBUG
  Serial.print(F("Temperature: ")); Serial.println(temperature);
  Serial.print(F("Pressure: ")); Serial.println(pressure);
#endif // DEBUG
}

// Read pitch, roll and tilt-compensated heading from LSM303 (°)
void readLsm303()
{
  const float alpha = 0.10;   // Alpha
  float fXa, fYa, fZa = 0;    // Low-pass filtered accelerometer variables

  if (lsm303.init())
  {
    lsm303.enableDefault();
    /*
      Calibration values: the default values of +/-32767 for each axis lead to an assumed
      magnetometer bias of 0. Use the Pololu LSM303 library Calibrate example program to
      determine appropriate values for each LSM303 sensor.

    */
    lsm303.m_min = (LSM303::vector<int16_t>)
    {
      //-32767, -32767, -32767   // Default
      //-334, -792, -713    // LSM303 Test Unit
      //-608, -705, -793    // LSM303 #1
      //-618, -699, -722    // LSM303 #2
      //-482, -813, -550    // LSM303 #3
      //-725, -664, -720    // LSM303 #4
      //-467, -860, -875    // LSM303 #5
      -546, -845,  -585    // LSM303 #6
    };
    lsm303.m_max = (LSM303::vector<int16_t>)
    {
      //+32767, +32767, +32767    // Default
      //+918, +450, +512    // LSM303 Test Unit
      //+500, +461, +597    // LSM303 #1
      //+635, +505, +360    // LSM303 #2
      //+670, +339, +511    // LSM303 #3
      //+589, +624, +483    // LSM303 #4
      //+819, +423, +326    // LSM303 #5
      +738, +402, +531    // LSM303 #6
    };

    // Apply low-pass filter to accelerometer data
    for (byte i = 0; i < 30; i++)   // 30 samples
    {
      lsm303.read();   // Read accelerometer and magnetometer data
      fXa = lsm303.a.x * alpha + (fXa * (1.0 - alpha));
      fYa = lsm303.a.y * alpha + (fYa * (1.0 - alpha));
      fZa = lsm303.a.z * alpha + (fZa * (1.0 - alpha));
      delay(5);
    }

    // Calculate orientation data
    float pitch = (atan2(-fXa, sqrt((long)fYa * fYa + (long)fZa * fZa)) * 180) / PI;
    float roll = (atan2(fYa, fZa) * 180) / PI;
    float heading = lsm303.heading((LSM303::vector<int>) {
      1, 0, 0   // PCB orientation
    });

    // Write orientation data to union
    message.pitch = pitch * 100;
    message.roll = roll * 100;
    message.heading = heading * 10;

#ifdef DEBUG
    Serial.print(F("pitch: ")); Serial.println(pitch);
    Serial.print(F("roll: ")); Serial.println(roll);
    Serial.print(F("heading: ")); Serial.println(heading);
#endif // DEBUG
  }
  else
  {
#ifdef DEBUG
    Serial.println(F("Error: Could not initialize LSM303!"));
#endif // DEBUG 
  }
}

// Read latitude, longitude, number of satellites and HDOP from GPS
void readGps()
{
#ifdef DEBUG
  unsigned long loopStartTime = millis();
#endif // DEBUG
  bool fixFound = false;
  bool charsSeen = false;
  byte month, day, hour, minute, second, hundredths;
  byte sentenceCounter = 0;
  int year, satellites;
  long latitude, longitude;
  unsigned long dateFix, locationFix, hdop;

  digitalWrite(GPS_EN, HIGH);   // Enable GPS
  ssGps.begin(9600);            // Open GPS software serial port
  delay(1000);

#ifdef DEBUG
  // Configure GPS
  ssGps.println("$PMTK220,1000*1F"); // Set NMEA port update rate to 1Hz
  delay(100);
  ssGps.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"); // Set NMEA sentence output frequencies to GGA and RMC
  delay(100);
  //ssGps.println("$PGCMD,33,1*6C"); // Enable antenna updates
  ssGps.println("$PGCMD,33,0*6D"); // Disable antenna updates
  delay(1000);
#endif // DEBUG

  // Look for GPS signal for up to 3 minutes
  for (unsigned long now = millis(); !fixFound && millis() - now < 3UL * 60UL * 1000UL;)
  {
    if (ssGps.available())
    {
      charsSeen = true;
      char c = ssGps.read();
#ifdef DEBUG  
      Serial.write(c);    // Echo NMEA sentences to serial
#endif // DEBUG

      if (tinyGps.encode(c))
      {
        sentenceCounter++;                                            // Track number of encoded NMEA sentences
        tinyGps.get_position(&latitude, &longitude, &locationFix);    // Latitude and longitude in millionths of a degree. Age of fix in milliseconds
        tinyGps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &dateFix);
        satellites = tinyGps.satellites();                            // Number of satellites used in last full GPGGA sentence
        hdop = tinyGps.hdop();                                        // Horizontal dilution of precision in hundreths
        fixFound = locationFix != TinyGPS::GPS_INVALID_FIX_TIME &&
                   dateFix != TinyGPS::GPS_INVALID_FIX_TIME &&
                   satellites != TinyGPS::GPS_INVALID_SATELLITES &&
                   hdop != TinyGPS::GPS_INVALID_HDOP &&
                   year != 2000;
      }
      if (fixFound && (sentenceCounter >= 20))   // Wait for NMEA sentence with sufficiently accurate GPS fix
      {
        // Write data to union
        message.latitude = latitude;
        message.longitude = longitude;
        message.satellites = satellites;
        message.hdop = hdop;
      }
      else
      {
        fixFound = false;
      }
    }
    ISBDCallback();

    // If we haven't seen any data in 10 seconds, then the wiring is wrong.
    if (!charsSeen && millis() - now > 10000)
      break;
  }
#ifdef DEBUG
  Serial.println(charsSeen ? fixFound ? F("A GPS fix was found!") : F("No GPS fix was found.") : F("Wiring error: No GPS data seen."));
  unsigned int loopDuration = (unsigned int)((millis() - loopStartTime) / 1000);
  Serial.print(F("loopDuration: ")); Serial.println(loopDuration);
  Serial.print(F("sentenceCounter: ")); Serial.println(sentenceCounter);
#endif // DEBUG
  digitalWrite(GPS_EN, LOW);    // Disable GPS
}

// Write data to external EEPROM
void writeEeprom()
{
#ifdef DEBUG
  printUnion();     // Print data stored in union
#endif  // DEBUG
  extEeprom.write(currentEepromAddress, message.bytes, messageSize);    // Write data stored in union to external EEPROM
  currentEepromAddress += messageSize;                                  // Increment external EEPROM address counter
  memset(message.bytes, 0x00, sizeof(message));                         // Clear data stored in union

#ifdef DEBUG
  Serial.print(F("currentEepromAddress: ")); Serial.println(currentEepromAddress);
  Serial.print(F("sizeof(message): ")); Serial.println(sizeof(message));
#endif // DEBUG
}

// Transmit data via the RockBLOCK 9603 satellite modem
void transmitData()
{
  unsigned long loopStartTime = millis();

  digitalWrite(ROCKBLOCK_EN, HIGH);   // Enable Pololu step-down voltage converter
  ssIridium.begin(19200);             // Open RockBLOCK software serial port

  if (isbd.begin() == ISBD_SUCCESS)
  {
    byte inBuffer[8];   // Buffer to store incomming RockBLOCK transmission (340 byte limit per message)
    size_t inBufferSize = sizeof(inBuffer);
    memset(inBuffer, 0x00, sizeof(inBuffer));   // Clear inBuffer array
    byte outBuffer[(messageSize * transmitInterval) * (retransmitCounter + 1)];   // Buffer to store outgoing RockBLOCK transmission
    size_t outBufferSize = sizeof(outBuffer);

    // Read and buffer data from external EEPROM in preparation for data transmission
    extEeprom.read(currentEepromAddress - outBufferSize, outBuffer, outBufferSize);   // Includes failed data transission attempts stored in external EEPROM

#ifdef DEBUG
    Serial.print(F("Message size: ")); Serial.println(outBufferSize);
#endif // DEBUG

    // Transmit and receieve data in binary format
    int err = isbd.sendReceiveSBDBinary(outBuffer, outBufferSize, inBuffer, inBufferSize);

    if (err == ISBD_SUCCESS)
    {
      retransmitCounter = 0;
      currentEepromAddress = 0;

      // Check for incoming message
      if (inBufferSize > 0)   // If no inbound message is available, inBufferSize = 0
      {
#ifdef DEBUG
        // Print inBuffer size and values of each incoming byte of data
        Serial.print(F("inBufferSize: ")); Serial.println(inBufferSize);
        for (int i = 0; i < inBufferSize; i++)
        {
          Serial.print(F("address: ")); Serial.print(i); Serial.print(F("\tvalue: ")); Serial.println(inBuffer[i], HEX);
        }
#endif // DEBUG

        // Recompose variables using bitshift
        unsigned long   alarmIntervalBuffer           = (((unsigned long)inBuffer[3] << 0) & 0xFF) + (((unsigned long)inBuffer[2] << 8) & 0xFFFF) + (((unsigned long)inBuffer[1] << 16) & 0xFFFFFF) + (((unsigned long)inBuffer[0] << 24) & 0xFFFFFFFF);
        unsigned int    transmitIntervalBuffer        = (((unsigned int)inBuffer[5] << 0) & 0xFF) + (((unsigned int)inBuffer[4] << 8) & 0xFFFF);
        unsigned int    maxRetransmitCounterBuffer    = (((unsigned int)inBuffer[7] << 0) & 0xFF) + (((unsigned int)inBuffer[6] << 8) & 0xFFFF);

        // Check if inBuffer data is valid
        if ((alarmIntervalBuffer > 900  && alarmIntervalBuffer <= 2678400) && (transmitIntervalBuffer > 0  && transmitIntervalBuffer <= 10) && (maxRetransmitCounterBuffer >= 0  && maxRetransmitCounterBuffer <= 10))
        {
          alarmInterval = alarmIntervalBuffer;
          transmitInterval = transmitIntervalBuffer;
          maxRetransmitCounter = maxRetransmitCounterBuffer;
        }
      }
    }
    else
    {
      retransmitCounter++;                            // Increment failed data transmission counter if err != ISBD_SUCCESS
      if (retransmitCounter > maxRetransmitCounter)   // If 340 byte limit is exceeded, discard all but last attempted message
      {
        retransmitCounter--;
      }
      if (currentEepromAddress >= maxEepromAddress)   // If external EEPROM address limit is exceeded, reset to zero
      {
        retransmitCounter = 0;
        currentEepromAddress = 0;

      }
#ifdef DEBUG
      Serial.print(F("Transmission failed with error code: ")); Serial.println(err);
#endif // DEBUG
    }
  }
  else
  {
    retransmitCounter++;                            // Increment failed data transmission counter if isbd.begin() != ISBD_SUCCESS
    if (retransmitCounter > maxRetransmitCounter)   // If 340 byte limit is exceeded, discard all but last attempted message
    {
      retransmitCounter--;
    }
    if (currentEepromAddress >= maxEepromAddress)    // If external EEPROM address limit is exceed, reset to zero
    {
      currentEepromAddress = 0;
      retransmitCounter = 0;
    }
#ifdef DEBUG
    Serial.println(F("Could not initialize RockBLOCK modem!"));
#endif // DEBUG
  }
  transmitDuration = (unsigned int)((millis() - loopStartTime) / 1000);
  message.transmitDuration = transmitDuration;

  isbd.sleep();                       // Sleep RockBLOCK
  ssIridium.end();                    // Close RockBLOCK software serial port
  digitalWrite(ROCKBLOCK_EN, LOW);    // Disable Pololu step-down voltage converter

#ifdef DEBUG
  Serial.print(F("transmitDuration: ")); Serial.println(transmitDuration);
  Serial.print(F("retransmitCounter: ")); Serial.println(retransmitCounter);
  Serial.print(F("currentEepromAddress: ")); Serial.println(currentEepromAddress);
  Serial.print(F("alarmInterval: ")); Serial.println(alarmInterval);
  Serial.print(F("transmitInterval: ")); Serial.println(transmitInterval);
  Serial.print(F("maxRetransmitCounter: ")); Serial.println(maxRetransmitCounter);
#endif DEBUG
}

// RockBLOCK callback function
bool ISBDCallback()   // This function can be repeatedly called during data transmission or GPS signal acquisition
{
#ifdef DEBUG
  pinMode(LED, OUTPUT);
  digitalWrite(LED, (millis() / 1000) % 2 == 1 ? HIGH : LOW);
  pinMode(LED, INPUT);
#endif // DEBUG

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 4000)
  {
    previousMillis = currentMillis;
    readBattery();    // Measure battery voltage
    wdt_reset();      // Pat the dog
  }
  return true;
}

// Configure Watchdog Timer
void configureWatchdog()
{
  cli();                                // Disable all interrupts
  wdt_reset();                          // Reset the watchdog timer
  MCUSR = 0;                            // Clear Reset Flags of MCU Status Register (MCUSR)
  WDTCSR |= (1 << WDCE) | (1 << WDE);   // Start timed sequence allowing alterations to Watchdog Timer Control Register (WDTCSR)
  // Set Watchdog Interrupt Enable (WDIE), clear Watchdog System Reset Enable (WDE) and set Watchdog Timer Prescaler WDP3 and WDP0 to select an 8 s timeout period
  WDTCSR = (1 << WDIE) | (0 << WDE) | (1 << WDP3) | (0 << WDP2) | (0 << WDP1) | (1 << WDP0);
  sei();                                // Enable all interrupts
}

// Watchdog interrupt service routine (ISR) to determine if system reset is required
ISR(WDT_vect)
{
  wdtIsrWasCalled = true;
  // Check if system reset is required
  if (sleeping == true)
  {
    wdt_reset();
    sleeping = false;
  }
  else
  {
    // Enable WDT System Reset Mode
    MCUSR = 0;                            // Clear Reset Flags of MCU Status Register (MCUSR)
    WDTCSR |= (1 << WDCE) | (1 << WDE);   // Start timed sequence allowing changes to Watchdog Timer Control Register (WDTCSR)
    // Clear Watchdog Interrupt Enable (WDIE), set Watchdog System Reset Enable (WDE) and clear Watchdog Timer Prescaler WDP3, WDP2, WDP1 and WDP0 to select a 16 ms timeout period
    WDTCSR = (0 << WDIE) | (1 << WDE) | (0 << WDP3) | (0 << WDP2) | (0 << WDP1) | (0 << WDP0); //
    while (1);                            // System reset will occur after 16 ms
  }
}

// Enable sleep and await RTC alarm interrupt
void goToSleep()
{
#ifdef DEBUG
  //Serial.println(F("sleepNow();"));
  Serial.flush();
#endif // DEBUG
  byte adcsra = ADCSRA;   // Save ADC Control and Status Register A (ADCSRA)
  ADCSRA = 0;             // Disable the ADC
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();                  // Disable all interrupts
  sleep_enable();         // Set the SE (sleep enable) bit
  wdt_reset();            // Pat the dog
  sleeping = true;        // Set sleep flag
  sleep_bod_disable();    // Disable BOD before going to sleep
  sei();                  // Enable all interrupts
  sleep_cpu();            // Put the device into sleep mode. Must be executed within 3 clock cycles in order to disable BOD
  sleep_disable();        // Clear the SE (sleep enable) bit. Program will execute from this point once awake
  ADCSRA = adcsra;        // Restore ADCSRA
}

// Print current time and date
void printDateTime(time_t t)
{
  Serial.print((day(t) < 10) ? "0" : ""); Serial.print(day(t), DEC); Serial.print('/');
  Serial.print((month(t) < 10) ? "0" : ""); Serial.print(month(t), DEC); Serial.print('/');
  Serial.print(year(t), DEC); Serial.print(' ');
  Serial.print((hour(t) < 10) ? "0" : ""); Serial.print(hour(t), DEC); Serial.print(':');
  Serial.print((minute(t) < 10) ? "0" : ""); Serial.print(minute(t), DEC); Serial.print(':');
  Serial.print((second(t) < 10) ? "0" : ""); Serial.println(second(t), DEC);
}

// LED activity indicator
void blink(byte flashes, int duration)
{
  wdt_reset();    // Pat the dog
  pinMode(LED, OUTPUT);
  for (int i = 0; i < flashes; i++)
  {
    digitalWrite(LED, HIGH);
    delay(duration);
    digitalWrite(LED, LOW);
    delay(duration);
  }
  pinMode(LED, INPUT);
}

// Print data stored in union
void printUnion()
{
#ifdef DEBUG
  Serial.println(F("------------------------------"));
  Serial.println(F("Union:"));
  Serial.println(F("------------------------------"));
  Serial.print(F("unixtime: ")); Serial.println(message.unixtime);
  Serial.print(F("temperature: ")); Serial.println(message.temperature);
  Serial.print(F("pressure: ")); Serial.println(message.pressure);
  Serial.print(F("pitch: ")); Serial.println(message.pitch);
  Serial.print(F("roll: ")); Serial.println(message.roll);
  Serial.print(F("heading: ")); Serial.println(message.heading);
  Serial.print(F("latitude: ")); Serial.println(message.latitude);
  Serial.print(F("longitude: ")); Serial.println(message.longitude);
  Serial.print(F("satellites: ")); Serial.println(message.satellites);
  Serial.print(F("hdop: ")); Serial.println(message.hdop);
  Serial.print(F("voltage: ")); Serial.println(message.voltage);
  Serial.print(F("messageCounter: ")); Serial.println(message.messageCounter);
  Serial.println(F("------------------------------"));
#endif // DEBUG
}
