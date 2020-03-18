/*
  Title:    Cryologger Ice Tracking Beacon (V3)
  Date:     January 12, 2020
  Author:   Adam Garbo

  Components:
  - SparkFun Qwiic Micro - SAMD21 Development Board
  - SparkFun Real Time Clock Module - RV-1805 (Qwiic)
  - SparkFun Atmospheric Sensor Breakout - BME280 (Qwiic)
  - SparkFun GPS Breakout - ZOE-M8Q (Qwiic)
  - SparkFun Qwiic Iridium 9603N
  - Maxtena M1621HCT-P-SMA Iridium antenna
  - Maxtena M1516HCT-P-SMA GNSS antenna
  - SparkFun 9DoF Sensor Stick
  - SparkFun Buck-Boost Converter

  Comments:
  - Code is currently under development for the next iteration of the Cryologger iceberg
  tracking beacon to be deployed during the 2020 Amundsen Expedition.
*/

// Libraries
#include <ArduinoLowPower.h>                // https://github.com/arduino-libraries/ArduinoLowPower
#include <IridiumSBD.h>                     // https://github.com/PaulZC/IridiumSBD
#include <SparkFunBME280.h>                 // https://github.com/sparkfun/SparkFun_BME280_Arduino_Library
#include <SparkFunLSM9DS1.h>                // https://github.com/sparkfun/SparkFun_LSM9DS1_Arduino_Library
#include <SparkFun_RV1805.h>                // https://github.com/sparkfun/SparkFun_RV-1805_Arduino_Library
#include <SparkFun_Ublox_Arduino_Library.h> // https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library
#include <TimeLib.h>                        // https://github.com/PaulStoffregen/Time
#include <Wire.h>                           // https://www.arduino.cc/en/Reference/Wire

// Defined constants
#define Serial        SerialUSB   // Required only by SparkFun Qwiic Micro 
#define IridiumWire   Wire
#define GPS_INT_PIN   A5          // Controls power to ZOE-M8Q
#define RTC_INT_PIN   4
#define DEBUG         true        // Output debugging messages to Serial Monitor
#define DIAGNOSTICS   true        // Output Iridium diagnostic messages to Serial Monitor

// Object instantiations
BME280        bme280;             // I2C Address: 0x77
IridiumSBD    modem(IridiumWire); // I2C Address: 0x63
RV1805        rtc;                // I2C Address: 0x69
SFE_UBLOX_GPS gps;                // I2C Address: 0x42

// User defined global variable declarations
uint32_t      alarmInterval         = 120;     // Sleep duration (in seconds) between data sample acquisitions (default = 3600 seconds/1 hour)
uint16_t      transmitInterval      = 10;       // Number of message to be included in each RockBLOCK transmission (340 byte limit)
uint16_t      maxRetransmitCounter  = 1;        // Maximum number of failed data transmissions to reattempt in a single message (340 byte limit)

// Global variable and constant declarations
bool          ledState              = LOW;      // LED toggle flag for blink() function
volatile bool sleeping              = false;    // Watchdog sleep flag
volatile bool alarmIsrFlag          = false;    // Alarm interrupt service routine flag
uint8_t       valFix                = 0;        // ZOE-M8Q fix counter
uint8_t       maxValFix             = 5;        // ZOE-M8Q max fix
uint8_t       transmitBuffer[340]   = {};       // Qwiic 9603N transmission buffer
uint16_t      messageCounter        = 0;        // Qwiic 9603N transmitted message counter
uint16_t      retransmitCounter     = 0;        // Qwiic 9603N failed data transmission counter
uint16_t      transmitCounter       = 0;        // Qwiic 9603N transmission interval counter
uint32_t      previousMillis        = 0;
time_t        alarmTime, unixtime;
tmElements_t  tm;

// UBX-CFG-PM2
uint8_t ubxCfgPm2[] = {
  0xB5, 0x62, 0x06, 0x3B, 0x2C, 0x00, 0x01, 0x06, 0x00, 0x00, 0x6E, 0x00, 0x42, 0x01,
  0xE8, 0x03, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x4F, 0xC1, 0x03, 0x00, 0x86, 0x02, 0x00, 0x00,
  0xFE, 0x00, 0x00, 0x00, 0x64, 0x40, 0x01, 0x00, 0xB2, 0x7A
};

// Structure and union to store and send data byte-by-byte via RockBLOCK
typedef union {
  struct {
    uint32_t  unixtime;           // Unix epoch                     (4 bytes)
    int16_t   temperature;        // Temperature (°C)               (2 bytes)
    uint16_t  humidity;           // Humidity (%)                   (2 bytes)
    uint16_t  pressure;           // Pressure (hPa)                 (2 bytes)
    //int16_t   pitch;              // Pitch (°)                    (2 bytes)
    //int16_t   roll;               // Roll (°)                     (2 bytes)
    //uint16_t  heading;            // Tilt-compensated heading (°) (2 bytes)
    int32_t   latitude;           // Latitude (DD)                  (4 bytes)
    int32_t   longitude;          // Longitude (DD)                 (4 bytes)
    uint8_t   satellites;         // # of satellites                (1 byte)
    uint16_t  pdop;               // PDOP                           (2 bytes)
    uint8_t   fix;                // Fix                            (1 byte)
    //uint16_t  voltage;            // Battery voltage (mV)         (2 bytes)
    uint16_t  transmitDuration;   // Previous message duration      (2 bytes)
    uint16_t  messageCounter;     // Message counter                (2 bytes)
  } __attribute__((packed));                                        // (33-byte message)
  uint8_t bytes[];
} SBDMESSAGE;

SBDMESSAGE message;
size_t messageSize = sizeof(message);   // Size (in bytes) of data message to be transmitted

// setup()
void setup() {

  // Pin assignments
  pinMode(GPS_INT_PIN, OUTPUT);     // SparkFun ZOE-M8Q interrupt pin to control power
  pinMode(LED_BUILTIN, OUTPUT);     // SparkFun Qwiic Micro blue LED
  digitalWrite(GPS_INT_PIN, HIGH);  // Enable power to GPS
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  while (!Serial);      // Prevent execution of script until Serial Monitor is open
  //delay(10000);         // Delay to allow for opening of Serial Monitor

  Serial.println(F("Cryologger Iceberg Tracking Beacon V3"));
  Serial.println(F("-------------------------------------"));

  // Configure Watchdog Timer
  configureWatchdog();

  // I2C Configuration
  Wire.begin();           // Initialize I2C
  Wire.setClock(400000);  // Set I2C clock speed to 400kHz

  // SparkFun RV-1805 RTC Configuration
  if (rtc.begin()) {
    Serial.println("SparkFun RV-1805 RTC detected.");
    //rtc.disableTrickleCharge();           // Disable capacitor trickle charger
    rtc.set24Hour();                      // Set RTC to 24-hour format
    rtc.setAlarm(0, 0, 0, 0, 0);          // Set the alarm (seconds, minutes, hours, day, month)
    rtc.setAlarmMode(6);                  // Set alarm mode to seconds match (once per minute))
    rtc.enableInterrupt(INTERRUPT_AIE);   // Enable the Alarm Interrupt

    // Configure and attach interrupt on the CLK/INT pin
    pinMode(RTC_INT_PIN, INPUT_PULLUP);
    LowPower.attachInterruptWakeup(RTC_INT_PIN, alarmIsr, FALLING);

    // Update time variable from RTC registers
    rtc.updateTime();
    Serial.println(rtc.stringTimeStamp()); // Print date and time in ISO 8601 format
  }
  else {
    Serial.println(F("SparkFun RV-1805 RTC not connected! Please check wiring. Halting."));
    while (1);
  }

  // SparkFun BME280 Configuration
  if (bme280.beginI2C()) {
    Serial.println(F("SparkFun BME280 detected."));
    bme280.setMode(MODE_SLEEP); // Enter sleep mode
  }
  else {
    Serial.println(F("SparkFun BME280 not connected! Please check wiring. Halting."));
    while (1);
  }

  // SparkFun Qwiic Iridium 9603N Configuration
  if (modem.isConnected()) {
    Serial.println(F("SparkFun Qwiic Iridium 9603N detected."));
    modem.adjustATTimeout(15);                            // Set AT timeout (Default = 20 seconds)
    modem.adjustSendReceiveTimeout(300);                  // Set send/receive timeout (Default = 300 seconds)
    modem.enable841lowPower(true);                        // Enable ATtiny841 low-power mode
  }
  else {
    Serial.println(F("Qwiic Iridium 9603N not connected! Please check wiring. Halting."));
    while (1);
  }

  // SparkFun ZOE-M8Q Configuration
  if (gps.begin() == true) {
    gps.setI2COutput(COM_TYPE_UBX); // Set I2C port to output UBX only (turn off NMEA noise)
    //gps.sendI2cCommand(gps.processUBX(ubxCfgPm2),100);
    gps.saveConfiguration();        // Save current settings to flash and BBR
    Serial.println(F("SparkFun ZOE-M8Q detected."));
  }
  else {
    Serial.println(F("u-blox ZOE-M8Q not detected at default I2C address. Please check wiring. Halting."));
    while (1);
  }
} // End setup()

// Loop
void loop() {

  // Check if alarm interrupt service routine was triggered
  if (alarmIsrFlag) {

    // Confirm alarm flag was set and not a false trigger (will reset flag if set)
    if (rtc.status() & 0x04) { // Alarm Flag bit in register 0Fh will be set to 1: 0b00000100 (0x04)

#if DEBUG
      Serial.print("Alarm triggered: ");

#endif

      // Read the RTC
      readRv1805();

      // Pet the Watchdog Timer
      resetWatchdog();

      // Read BME280
      readBme280();

      // Read GPS
      readGps();

      // Write data to buffer
      writeBuffer();

      // Check if data is to be transmitted
      if (transmitCounter == transmitInterval) {
        transmitData();       // Transmit data
        transmitCounter = 0;  // Reset transmit counter
      }

      // Set RTC alarm
      alarmTime = unixtime + alarmInterval; // Calculate next alarm

      // Ensure alarm is not set in the past
      if (alarmTime <= unixtime) {
        unixtime = getUnixtime();
        alarmTime = unixtime + alarmInterval; // Calculate new alarm
      }
      rtc.setAlarm(0, minute(alarmTime), hour(alarmTime), day(alarmTime), month(alarmTime));
      //rtc.setAlarmMode(2);                // Set alarm mode to seconds, minutes, hours and date match (once per month))
      rtc.setAlarmMode(5);                // Set alarm mode to seconds and minutes match (once per hour))
      rtc.enableInterrupt(INTERRUPT_AIE); // Ensure Alarm Interrupt is enabled
      Serial.print(F("Next alarm: ")); Serial.println(alarmTime);
      printDatetime(alarmTime);
    }
    alarmIsrFlag = false; // Clear alarm interrupt service routine flag
  }
  sleeping = true; // Watchdog Timer
  blinkLed(1, 50);
  delay(250);
  //LowPower.deepSleep(); // Enter deep sleep
}

// Read SparkFun RV-1805 RTC
void readRv1805() {

  // Loop timer
  uint32_t loopStartTime = millis();

  // Convert to Unix epoch (tm to time_t)
  unixtime = getUnixtime();

  // Write data to union
  message.unixtime = unixtime;

  Serial.println(rtc.stringTimeStamp()); // Print date and time in ISO 8601 format

  // Loop timer
  uint32_t loopEndTime = millis() - loopStartTime;
  Serial.print("readRv1805() function execution: "); Serial.print(loopEndTime); Serial.println(F(" ms"));
}

// Convert datetime to Unix epoch time
uint32_t getUnixtime() {

  // Update the local array with the RTC registers
  rtc.updateTime();

  // Convert to Unix epoch (tm to time_t)
  tmElements_t tm;
  tm.Second = rtc.getSeconds();
  tm.Minute = rtc.getMinutes();
  tm.Hour   = rtc.getHours();
  tm.Day    = rtc.getDate();
  tm.Month  = rtc.getMonth();
  tm.Year   = rtc.getYear() + 30; // Offset from 2000 - 1970
  time_t t = makeTime(tm);

  return t;
}

// RTC alarm interrupt service routine
void alarmIsr() {
  alarmIsrFlag = true; // Set alarm flag
}

// Read SparkFun BME280
void readBme280() {

  uint32_t loopStartTime = millis();  // Loop timer

  bme280.setMode(MODE_FORCED);  // Wake up sensor and take reading

  float temperature = bme280.readTempC();
  float humidity = bme280.readFloatHumidity();
  float pressure = bme280.readFloatPressure();

  // Write data to union
  message.temperature = temperature * 100;
  message.humidity = humidity * 100;
  message.pressure = pressure / 10;

  uint32_t loopEndTime = millis() - loopStartTime;
  Serial.print("readBme280() function execution: "); Serial.print(loopEndTime); Serial.println(F(" ms"));

#if DEBUG
  Serial.print(F("Temperature: ")); Serial.print(temperature, 2);
  Serial.print(F(" Humidity: ")); Serial.print(humidity, 2);
  Serial.print(F(" Pressure: ")); Serial.println(pressure, 2);
#endif
}


// Read SparkFun ZOE-M8Q
void readGps() {

  // Loop timer
  uint32_t loopStartTime = millis();

  // Enable GPS
  digitalWrite(GPS_INT_PIN, HIGH); // Set INT pin HIGH

  // Begin listening to the GPS
  Serial.println(F("Beginning to listen for GPS traffic..."));

  // Look for GPS signal for up to 2 minutes
  while ((valFix != maxValFix) && millis() - loopStartTime < 2UL * 60UL * 1000UL) {
#ifdef DEBUG
    char datetime[20]; // GNSS date time buffer
    snprintf(datetime, sizeof(datetime), "%04u-%02d-%02d %02d:%02d:%02d",
             gps.getYear(), gps.getMonth(), gps.getDay(),
             gps.getHour(), gps.getMinute(), gps.getSecond());

    float latitude = gps.getLatitude() / 10000000.0;
    float longitude = gps.getLongitude() / 10000000.0;
    float pdop = gps.getPDOP() / 100.0;
    uint8_t fix = gps.getFixType();
    uint8_t satellites = gps.getSIV();

    Serial.print("DATETIME: "); Serial.print(datetime);
    Serial.print(" LAT: "); Serial.print(latitude, 6);
    Serial.print(" LON: "); Serial.print(longitude, 6);
    Serial.print(" SAT: "); Serial.print(satellites);
    Serial.print(" FIX: "); Serial.print(fix);
    Serial.print(" PDOP: "); Serial.println(pdop, 2);
#endif

    // Did we get a GPS fix?
    if (gps.getFixType() > 0) {
      valFix += 1; // Increment valFix
    }

    // Have enough valid GNSS fixes been collected?
    if (valFix == maxValFix) {
      Serial.println(F("A GPS fix was found!"));
      /*
            // Set the RTC time
            // Hundredths, seconds, minutes, hours, day, month, year, weekday
            rtc.setTime(0, gps.getSecond(), gps.getMinute(), gps.getHour(),
                        gps.getDay(), gps.getMonth(), gps.getYear(), 0);

            rtc.updateTime();
            Serial.print("RTC time set: "); Serial.println(rtc.stringTimeStamp());
      */

      // Write data to union
      message.latitude = gps.getLatitude();
      message.longitude = gps.getLongitude();
      message.satellites = gps.getSIV();
      message.pdop = gps.getPDOP();
      message.fix = gps.getFixType();
      break;
    }
    ISBDCallback();
  }

  // Did we get a GPS fix?
  if (valFix < maxValFix) {
    Serial.println(F("No GPS fix was found."));
  }

  // Reset valFix counter
  valFix = 0;

  uint32_t loopEndTime = millis() - loopStartTime;
  Serial.print("readGps() function execution: "); Serial.print(loopEndTime); Serial.println(F(" ms"));

  // Disable GPS
  digitalWrite(GPS_INT_PIN, LOW);

} // End readGps()

// Write union data to transmit buffer in preparation of data transmission
void writeBuffer() {
  messageCounter++;                         // Increment message counter
  message.messageCounter = messageCounter;  // Write message counter data to union
  transmitCounter++;                        // Increment data transmission counter

  // Concatenate current message with existing message(s) stored in transmit buffer
  memcpy(transmitBuffer + (sizeof(message) * (transmitCounter + (retransmitCounter * transmitInterval) - 1)), message.bytes, sizeof(message)); // Copy message to transmit buffer

#if DEBUG
  printUnion();
  //printUnionBinary(); // Print union/structure in hex/binary
  //printTransmitBuffer();  // Print transmit buffer in hex/binary
#endif
}

// Transmit data
void transmitData() {

  uint32_t loopStartTime = millis(); // Loop timer
  int16_t err;

  // Enable the supercapacitor charger
  Serial.println(F("Enabling the supercapacitor charger..."));
  modem.enableSuperCapCharger(true);

  // Wait for the supercapacitor charger PGOOD signal to go high
  while (!modem.checkSuperCapCharger()) {
    Serial.println(F("Waiting for supercapacitors to charge..."));
    delay(500); // CHANGE TO NON BLOCKING LOOP
  }
  Serial.println(F("Supercapacitors charged!"));

  // Enable power for the Qwiic 9603N
  Serial.println(F("Enabling 9603N power..."));
  modem.enable9603Npower(true);

  // Begin satellite modem operation
  Serial.println(F("Starting modem..."));
  err = modem.begin();
  if (err == ISBD_SUCCESS) {
    uint8_t inBuffer[240];  // Buffer to store incoming transmission (340 byte limit)
    size_t inBufferSize = sizeof(inBuffer);
    memset(inBuffer, 0x00, sizeof(inBuffer)); // Clear inBuffer array

    /*
        // Test the signal quality
        int signalQuality = -1;
        err = modem.getSignalQuality(signalQuality);
        if (err != ISBD_SUCCESS) {
          Serial.print(F("SignalQuality failed: error "));
          Serial.println(err);
          return;
        }
      Serial.print(F("On a scale of 0 to 5, signal quality is currently: "));
      Serial.println(signalQuality);
    */

    // Transmit and receieve data in binary format
    err = modem.sendReceiveSBDBinary(transmitBuffer, (sizeof(message) * (transmitCounter + (retransmitCounter * transmitInterval))), inBuffer, inBufferSize);

    // Check if transmission was successful
    if (err == ISBD_SUCCESS) {
      retransmitCounter = 0;
      memset(transmitBuffer, 0x00, sizeof(transmitBuffer)); // Clear transmit buffer array

      Serial.println(F("Success!"));
      digitalWrite(LED_BUILTIN, HIGH);
    }
    else {
      Serial.print(F("Transmission failed: error "));
      Serial.println(err);
    }
  }
  else {
    Serial.print(F("Begin failed: error "));
    Serial.println(err);
    if (err == ISBD_NO_MODEM_DETECTED) {
      Serial.println(F("No modem detected: check wiring."));
    }
    return;
  }

  // Store message in transmit buffer if transmission or modem begin fails
  if (err != ISBD_SUCCESS) {
    retransmitCounter++;
    // Reset counter if reattempt limit is exceeded
    if (retransmitCounter >= maxRetransmitCounter) {
      retransmitCounter = 0;
      memset(transmitBuffer, 0x00, sizeof(transmitBuffer));   // Clear transmitBuffer array
    }
  }

  // Power down the modem
  Serial.println(F("Putting the Qwiic 9603N to sleep."));
  err = modem.sleep();
  if (err != ISBD_SUCCESS) {
    Serial.print(F("sleep failed: error "));
    Serial.println(err);
  }

  // Disable 9603N power
  Serial.println(F("Disabling 9603N power..."));
  modem.enable9603Npower(false);

  // Disable the supercapacitor charger
  Serial.println(F("Disabling the supercapacitor charger..."));
  modem.enableSuperCapCharger(false);

  digitalWrite(LED_BUILTIN, LOW);
  uint32_t transmitDuration = millis() - loopStartTime;
  message.transmitDuration = transmitDuration / 1000;
  uint32_t loopEndTime = millis() - loopStartTime;

  Serial.print(F("transmitData() function execution: ")); Serial.print(loopEndTime); Serial.println(F(" ms"));
  Serial.print(F("transmitDuration: ")); Serial.println(transmitDuration / 1000);
  Serial.print(F("retransmitCounter: ")); Serial.println(retransmitCounter);

} // End transmitData()

// Blink LED (non-blocking)
void blinkLed(uint8_t flashes, uint16_t interval) {
  uint8_t i = 0;
  while (i <= flashes) {
    uint32_t currentMillis = millis();
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
} // End blinkLed()

// RockBLOCK callback function
bool ISBDCallback() {
  resetWatchdog(); // Pet the Watchdog
  blinkLed(1, 500);
  return true;
} // End ISBDCallback()

// Callback to sniff the conversation with the Iridium modem
void ISBDConsoleCallback(IridiumSBD * device, char c) {
#if DIAGNOSTICS
  Serial.write(c);
#endif
} // End ISBDConsoleCallback()

// Callback to to monitor Iridium modem library's run state
void ISBDDiagsCallback(IridiumSBD * device, char c) {
#if DIAGNOSTICS
  Serial.write(c);
#endif
} // End ISBDDiagsCallback()

// Print current time and date.
void printDatetime(time_t t) {
  char dateBuffer[25];
  snprintf(dateBuffer, sizeof(dateBuffer), "%04u-%02d-%02d %02d:%02d:%02d",
           year(t), month(t), day(t), hour(t), minute(t), second(t));
  Serial.println(dateBuffer);
}

// Print union/structure
void printUnion() {
  Serial.println(F("-----------------------------------"));
  Serial.println(F("Union/structure"));
  Serial.println(F("-----------------------------------"));
  Serial.print(F("unixtime:\t\t")); Serial.println(message.unixtime);
  Serial.print(F("temperature:\t\t")); Serial.println(message.temperature);
  Serial.print(F("pressure:\t\t")); Serial.println(message.pressure);
  Serial.print(F("humidity:\t\t")); Serial.println(message.humidity);
  //Serial.print(F("pitch:\t\t\t")); Serial.println(message.pitch);
  //Serial.print(F("roll:\t\t\t")); Serial.println(message.roll);
  //Serial.print(F("heading:\t\t")); Serial.println(message.heading);
  Serial.print(F("latitude:\t\t")); Serial.println(message.latitude);
  Serial.print(F("longitude:\t\t")); Serial.println(message.longitude);
  Serial.print(F("satellites:\t\t")); Serial.println(message.satellites);
  Serial.print(F("pdop:\t\t\t")); Serial.println(message.pdop);
  Serial.print(F("fix:\t\t\t")); Serial.println(message.fix);
  //Serial.print(F("voltage:\t\t")); Serial.println(message.voltage);
  Serial.print(F("transmitDuration:\t")); Serial.println(message.transmitDuration);
  Serial.print(F("messageCounter:\t\t")); Serial.println(message.messageCounter);
  Serial.println(F("-----------------------------------"));
} // End printUnion()

// Print contents of union/structure
void printUnionBinary() {
  Serial.println(F("Union/structure "));
  Serial.println(F("-----------------------------------"));
  Serial.println(F("Byte\tHex\tBinary"));
  for (uint16_t i = 1; i <= sizeof(message); ++i) {
    Serial.print(i);
    Serial.print("\t");
    Serial.print(message.bytes[i], HEX);
    Serial.print("\t");
    Serial.println(message.bytes[i], BIN);
  }
  Serial.println(F("-----------------------------------"));
} // End printUnionBinary()

// Print contents of transmiff buffer array
void printTransmitBuffer() {
  Serial.println(F("Transmit buffer"));
  Serial.println(F("-----------------------------------"));
  Serial.println(F("Byte\tHex\tBinary"));
  for (uint16_t i = 1; i <= 340; i++) {
    Serial.print(i);
    Serial.print("\t");
    Serial.print(transmitBuffer[i], HEX);
    Serial.print("\t");
    Serial.println(transmitBuffer[i], BIN);
  }
} // End printTransmitBuffer()

// Configure the WDT to perform a system reset if loop() blocks for more than 8-16 seconds
void configureWatchdog() {

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

  WDT->EWCTRL.bit.EWOFFSET = 0xA;                   // Set the Early Warning Interrupt Time Offset to 8 seconds // REG_WDT_EWCTRL = WDT_EWCTRL_EWOFFSET_8K;
  WDT->INTENSET.bit.EW = 1;                         // Enable the Early Warning interrupt                       // REG_WDT_INTENSET = WDT_INTENSET_EW;
  WDT->CONFIG.bit.PER = 0xB;                        // Set the WDT reset timeout to 16 seconds                  // REG_WDT_CONFIG = WDT_CONFIG_PER_16K;
  WDT->CTRL.bit.ENABLE = 1;                         // Enable the WDT in normal mode                            // REG_WDT_CTRL = WDT_CTRL_ENABLE;
  while (WDT->STATUS.bit.SYNCBUSY);                 // Await synchronization of registers between clock domains

  // Configure and enable WDT interrupt
  NVIC_DisableIRQ(WDT_IRQn);
  NVIC_ClearPendingIRQ(WDT_IRQn);
  NVIC_SetPriority(WDT_IRQn, 0);                    // Top priority
  NVIC_EnableIRQ(WDT_IRQn);
} // End configureWatchdog()

// Pet the Watchdog Timer
void resetWatchdog() {
  WDT->CLEAR.bit.CLEAR = 0xA5;        // Clear the Watchdog Timer and restart time-out period //REG_WDT_CLEAR = WDT_CLEAR_CLEAR_KEY;
  while (WDT->STATUS.bit.SYNCBUSY);   // Await synchronization of registers between clock domains
} // End resetWatchdog()

// Watchdog Timer interrupt service routine
void WDT_Handler() {
  if (sleeping) {
    sleeping = false;
    WDT->INTFLAG.bit.EW = 1;          // Clear the Early Warning interrupt flag //REG_WDT_INTFLAG = WDT_INTFLAG_EW;
    WDT->CLEAR.bit.CLEAR = 0xA5;      // Clear the Watchdog Timer and restart time-out period //REG_WDT_CLEAR = WDT_CLEAR_CLEAR_KEY;
    while (WDT->STATUS.bit.SYNCBUSY); // Await synchronization of registers between clock domains
  }
  else {
    WDT->CTRL.bit.ENABLE = 0;       // DEBUG: Disable Watchdog
    digitalWrite(13, HIGH);         // DEBUG: Turn on LED to indicate Watchdog trigger
    while (true);                     // Force Watchdog Timer to reset the system
  }
} // End WDT_Hander()
