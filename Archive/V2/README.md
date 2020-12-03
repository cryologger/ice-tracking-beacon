# Cryologger - Iceberg Tracking Beacon (ITB) v2.0
![Image](https://github.com/adamgarbo/Cryologger_Iceberg_Tracking_Beacon/blob/master/Archive/V2/Documentation/Images/DSC_2351.JPG)

## Background
Icebergs and ice islands represent significant hazards to marine navigation and offshore infrastructure at a time when demand for access to Canada’s Arctic waters is increasing.

There is a growing demand for in situ iceberg tracking data to monitor their drift trajectory and improve predictions of ice hazard occurrence and behaviour, yet the high cost of commercial tracking devices often prevents monitoring at optimal spatial and temporal resolutions.

## Objective
The goal of this project is to determine if the Cryologger, a tracking beacon based on inexpensive, open-source hardware and software, can provide a reliable and cost-effective platform for monitoring the drift of icebergs and ice islands in the Canadian Arctic.

## Design
The Cryologger is based on the open-source Arduino platform (www.arduino.cc) and built using low-cost, do-it-yourself electronics that can be easily modified to meet the needs of the end-user. Code was written using the Arduino Integrated Development Environment (IDE) and benefits from the availability of community-generated libraries.

Planned for extended deployments in harsh Arctic conditions, the Cryologger can provide long-term measurements of GPS position, temperature, pressure, pitch, roll, tilt-compensated heading and battery voltage. Data are transmitted over the Iridium satellite network at specified intervals and can be remotely updated based on the desired sampling frequency. Collected data are made freely available and can be viewed in near-real time at https://cryologger.org.

#### Materials

| Component | Product | Cost (USD) |
| --- | --- | :---: | 
| Satellite transceiver | Rock7 RockBLOCK 9603 | $249.95 |
| Processor | Adafruit Feather M0 Basic Proto | $19.95 |
| Real-time clock | Adafruit DS3231 Precision RTC FeatherWing | $13.95 |
| GNSS | Adafruit Ultimate GPS FeatherWing | $39.95 |
| IMU | Pololu LSM303D 3D Compass and Accelerometer | $7.95 |
| Voltage Regulator | Pololu D36V6F3 3.3V 600mA Step-Down Voltage Regulator | $4.95 |
| Battery |	Tadiran	Lithium Pulses Plus 7.2V 38Ah 273.6Wh | $100 |
| Enclosure | Nanuk 903 | $27.95 |

#### Measurements
| Variable | Unit | Comments |
| --- | :---: | --- |
| Datetime  |   | YYYY-MM-DD HH:MM:SS |
| Temperature | °C  | Internal temperature |
| Pitch | °|  |
| Roll | ° |  |
| Heading | °  | Tilt-compensated heading (0-360°) |
| Latitude | DD |  |
| Longitude | DD |  |
| Satellites | # | Number of satellites in view  |
| HDOP |  | Horizonal dilution of precision |
| Voltage | V | Battery voltage |
| Transmit duration  | s | Length of Iridium transmission  |
| Message counter |  | Number of transmitted messages |

## Repository Contents

* **/Archive** - Contains information on previous versions of the Cryologger design.

* **/Software** - Contains the Arduino code.

* **/Bill of Materials** - Information on all components used construction and their associated costs.

* **/Documentation** - All project documents

* **/Hardware** - Autodesk EAGLE schematics and Fusion 360 design files

## Documentation
* Currently in progress

## License Information
This project is distributed under the GNU General Public License v3.0

Cheers,

**Adam**
