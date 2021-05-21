# Cryologger - Iceberg Tracking Beacon (ITB)
![Image](https://github.com/adamgarbo/Cryologger_Iceberg_Tracking_Beacon/blob/main/Archive/v2.0/Images/2019_300434063392070.JPG)

## Background
Icebergs and ice islands represent significant hazards to marine navigation and offshore infrastructure at a time when demand for access to Canada’s Arctic waters is increasing. There is a growing demand for in situ iceberg tracking data to monitor their drift trajectory and improve predictions of ice hazard occurrence and behaviour, yet the high cost of commercial tracking devices often prevents monitoring at optimal spatial and temporal resolutions.

## Objective
The goal of this project is to determine if the Cryologger, a tracking beacon based on inexpensive, open-source hardware and software, can provide a reliable and cost-effective platform for monitoring the drift of icebergs and ice islands in the Canadian Arctic.

## Design
The Cryologger is based on the open-source Arduino platform (www.arduino.cc) and built using low-cost, do-it-yourself electronics that can be easily modified to meet the needs of the end-user. Code was written using the Arduino Integrated Development Environment (IDE) and benefits from the availability of community-generated libraries.

Planned for extended deployments in harsh Arctic conditions, the Cryologger can provide long-term measurements of GPS position, temperature, pressure, pitch, roll, tilt-compensated heading and battery voltage. Data are transmitted over the Iridium satellite network at specified intervals and can be remotely updated based on the desired sampling frequency. Collected data are made freely available and can be viewed in near-real time at https://cryologger.org.

### v3.0 Prototype
* v3.0 of the Cryologger is currently under development and will also be built using Adafruit components. It will build on the success of v2.0, with a custom PCB to greatly simplify assembly.

#### Materials 

| Component | Product | Cost (USD) | URL |
| --- | --- | :---: |  --- |
| Satellite transceiver | Rock7 RockBLOCK 9603 | $249.95 | http://www.rock7mobile.com/products-rockblock-9603 |
| Satellite antenna | Maxtena M1621HCT-P-SMA | $50.00 | https://www.richardsonrfpd.com/Products/Product/M1621HCT-P-SMA |
| Processor | Adafruit Feather M0 Basic Proto | $19.95 | https://www.adafruit.com/product/2772 |
| GPS | Adafruit Ultimate GPS FeatherWing | $39.95 | https://www.adafruit.com/product/3133 |
| IMU | Adafruit LSM6DS33 + LIS3MDL - 9 DoF IMU | $9.95 | https://www.adafruit.com/product/4485 |
| Sensor | Adafruit DPS310 Precision Barometric Pressure Sensor | $6.95 | https://www.adafruit.com/product/4494 |
| Voltage Regulator | Pololu D36V6F3 3.3V 600mA Step-Down Voltage Regulator | $4.95 | https://www.pololu.com/product/3791 |
| Voltage Regulator | Pololu D36V6F5 5V 600mA Step-Down Voltage Regulator | $4.95 | https://www.pololu.com/product/3792 |
| Enclosure |  | ~$50.00 | |
| Battery | Tadiran | ~$150.00 | |


#### Custom PCB
<img width="920" alt="Screen Shot 2021-05-21 at 1 04 29 PM" src="https://user-images.githubusercontent.com/22924092/119173673-94f43f00-ba35-11eb-9bf8-35857b2f1c34.png">


#### Measurements
| Variable | Unit | Comments |
| --- | :---: | --- |
| Datetime  |   | YYYY-MM-DD HH:MM:SS |
| Temperature | °C  | Internal temperature |
| Pressure | hPa | Internal pressure |
| Pitch | °|  |
| Roll | ° |  |
| Heading | °  | Tilt-compensated heading (0-360°) |
| Latitude | DD |  |
| Longitude | DD |  |
| Satellites | # | Number of satellites in view  |
| HDOP |  | Horizonal dilution of precision |
| Altitude | m | GPS altitude |
| Voltage | V | Battery voltage |
| Transmit duration  | s | Length of Iridium transmission  |
| Iteration counter |  | Number of program iterations |

## Repository Contents

* **/Archive** - Contains information on previous versions of the Cryologger design

* **/Software** - Contains the Arduino code

* **/Bill of Materials** - Information on all components used construction and their associated costs

* **/Documentation** - All project documents

* **/Hardware** - Autodesk EAGLE schematics and Fusion 360 design files

## Documentation
* Currently in progress

## License Information
This project is distributed under the GNU General Public License v3.0

Cheers,

**Adam**
