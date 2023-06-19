# Cryologger - Ice Tracking Beacon (ITB) Configuration
This guide provides step-by-step instructions on how to install the Arduino IDE and required board definitons and libraries, which are necessary for uploading code to the Cryologger ITB.

## Step 1: Download Arduino IDE  
* Navigate to https://www.arduino.cc/en/software and download the most recent version of the Arduino IDE.

![image](https://github.com/adamgarbo/cryologger-ice-tracking-beacon/assets/22924092/323c74a1-2ce1-4e39-a8eb-d39ad760a9a6)

## Step 2: Add Additional Boards Manager URL:
* In the Arduino IDE navigate to: Preferences
* Add the following Additional Boards Manager URL as shown in the sceenshot below:  
```https://adafruit.github.io/arduino-board-index/package_adafruit_index.json```
* Also during this step, check the "compile" and "upload" boxes for "Show verbose output during" 
![image](https://github.com/adamgarbo/cryologger-ice-tracking-beacon/assets/22924092/fe589849-2062-4941-9465-0b012f48ba62)

## Step 3: Install Board Definitions
* Navigate to: Tools > Boards > Boards Manager
* Search for: SAMD Boards
* Select and install version: Arduino SAMD Boards v1.8.13 
* Select and install version: Adafruit SAMD Boards v1.7.12
![image](https://github.com/adamgarbo/cryologger-ice-tracking-beacon/assets/22924092/f4d9af80-c4ba-4fab-958b-c35e1e4ac1e3)

## Step 4: Install Libraries
Several libraries are required by the Cryologger ice tracking beacons. These can downloaded using the Arduino IDE's Library Manager (requires Internet connection).

* Navigate to: Tools > Manage Libraries
* Search for and install the following libraries:

| Library Manager         | Version | GitHub Repository                                                   |
|-------------------------|---------|---------------------------------------------------------------------|
| Adafruit BME280         | 2.2.2   | https://github.com/adafruit/Adafruit_BME280_Library                 |
| Adafruit LSM303 Accel   | 1.1.6   | https://github.com/adafruit/Adafruit_LSM303_Accel                   |
| Adafruit LIS2MDL        | 2.1.4   | https://github.com/adafruit/Adafruit_LIS2MDL                        |
| Adafruit Unified Sensor | 1.1.9   | https://github.com/adafruit/Adafruit_Sensor                         |
| Arduino Low Power       | 1.2.2   | https://github.com/arduino-libraries/ArduinoLowPower                |
| IridiumSBD              | 3.0.6   | https://github.com/sparkfun/SparkFun_IridiumSBD_I2C_Arduino_Library |
| RTCZero                 | 1.6.0   | https://github.com/arduino-libraries/RTCZero                        |
| TimeLib                 | 1.6.1   | https://github.com/PaulStoffregen/Time                              |
| TinyGPSPlus             | 1.0.3   | https://github.com/mikalhart/TinyGPSPlus                            |

![image](https://github.com/adamgarbo/cryologger-ice-tracking-beacon/assets/22924092/dadb37fe-46c7-48af-ad3d-cbb4061d01a6)

## Step 5: Download Cryologger Ice Tracking Beacon Software
Code for the Cryologger ITB is made available on the following GitHub repository:
* https://github.com/adamgarbo/cryologger-ice-tracking-beacon
* Click on "Releases" and download the vX.X .zip file:

## Step 6: Test Program Compilation
* Navigate to the /Software/Arduino/cryologger_itb folder of the downloaded repository
* Double click on the `cryologger_itb.ino` file
* Click on the checkmark in the upper left corner of the Arduino IDE program window
* Watch debugging window for compilation errors
![image](https://github.com/adamgarbo/cryologger-ice-tracking-beacon/assets/22924092/c82fb874-a910-4be7-866c-09b41ea7edb9)

## Step 7: Connect Harware
* Connect to the Cryologger ITB using a micro USB cable.
* In the Arduino IDE click on "Select Board" and then "Adafruit Feather M0 (SAMD).
![image](https://github.com/adamgarbo/cryologger-ice-tracking-beacon/assets/22924092/0199d2f2-ca16-42ae-bb7f-c8cd82348479)
* If the board is not auto-populated, click on "Select other board and port..." and search for "Adafruit Feather M0":
* Be sure to select the appropriate serial port that is connected to the Arduino.
![image](https://github.com/adamgarbo/cryologger-ice-tracking-beacon/assets/22924092/dc0b3bcc-e7c9-4635-941f-46600e63a128)

## Step 7: Upload Program
* Once the code has compiled successfully, click on the right pointed arrow to upload the code
* Watch debugging window for compilation errors and/or success messages
![image](https://github.com/adamgarbo/cryologger-ice-tracking-beacon/assets/22924092/9b6e171d-9864-46d8-8cc3-003e7c313a0c)
