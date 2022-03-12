# Cryologger - Iceberg Tracking Beacon (ITB)
![2021_300434065734810](https://user-images.githubusercontent.com/22924092/158026368-e336462b-1e3f-46a9-8846-9ab7fdb8b459.JPG)

## 1.0 Introduction
Icebergs and ice islands represent significant hazards to marine navigation and offshore infrastructure at a time when demand for access to Canada’s Arctic waters is increasing. There is a growing demand for in situ iceberg tracking data to monitor their drift trajectory and improve predictions of ice hazard occurrence and behaviour, yet the high cost of commercial tracking devices often prevents monitoring at optimal spatial and temporal resolutions.

The goal of this project is to demonstrate that the Cryologger, a tracking beacon based on inexpensive, open-source hardware and software, can provide a reliable and cost-effective platform for monitoring the drift of icebergs and ice islands in the Canadian Arctic.

## 2.0 Methods

### 2.1 Design

The Cryologger is based on the open-source Arduino platform (www.arduino.cc) and built using low-cost, do-it-yourself electronics that can be easily modified to meet the needs of the end-user. Code was written using the Arduino Integrated Development Environment (IDE) and benefits from the availability of community-generated libraries.

Planned for extended deployments in harsh Arctic conditions, the Cryologger provides long-term measurements of GPS position, temperature, pressure, pitch, roll, tilt-compensated heading and battery voltage. Data are transmitted over the Iridium satellite network at specified intervals and can be remotely updated based on the desired sampling frequency. Collected data are made freely available and can be viewed in near-real time at https://cryologger.org.

#### 2.1.1 Changes in v3.0
Version 3.0 of the Cryologger iceberg tracking beacon builds on the success of v2.0 and is also based on the Adafruit ecosystem of components. A number of improvements to the design were made, including:
* 3.3 V power is now provided directly from a Pololu 3.3 V step-down voltage regulator, bypassing the Feather M0's onboard AP2112 LDO regulator (-55 μA).
* A dedicated 5 V step-down voltage regulator was added to power the RockBLOCK Iridium transceiver.
* The DS3231 real-time clock (RTC) was removed in favour of the SAMD21's internal RTC for all timekeeping and alarm functionality with periodic time synchronizations with the GPS.
* The LSM303 accelerometer/magnetometer was replaced with the LSM6DS33 + LIS3MDL IMU due to the sensor reaching its end-of-life (EOL).
* The temperature/pressure and IMU sensors are now powered directly through SAMD21 GPIO pins, allowing for power to be completely removed during sleep.
* A 2MΩ + 1 MΩ resistor divider is now used to measured the battery voltage (+2.4 μA).

#### 2.1.2 Custom PCB
Another major change in v3.0 is a custom carrier board PCB that designed to greatly simplify the assembly process. The PCB was designed in KiCad and fabricated by JLCPCB. 

<p align="left"><img src="https://user-images.githubusercontent.com/22924092/119173673-94f43f00-ba35-11eb-9bf8-35857b2f1c34.png" width="480"></p>
<p align="left"><b>Figure 1:</b> 3D rendering of Cryologger iceberg drift tracking beacon carrier board designed in KiCad.</p>

### 2.2 Bill of Materials 

**Table 1.** Bill of materials and associated costs for components used in the Cryologger iceberg tracking beacon v3.0.
| Component | Product | Cost (USD) |
| --- | --- | :---: |
| Satellite transceiver | [Rock7 RockBLOCK 9603](http://www.rock7mobile.com/products-rockblock-9603) | $249.95 |
| Satellite antenna | [Maxtena M1621HCT-P-SMA](https://maxtena.com/products/f-passive/m1621hct-p-sma-iridium-passive-antenna/) | $50.00 |
| Processor | [Adafruit Feather M0 Basic Proto](https://www.adafruit.com/product/2772) | $19.95 |
| GPS | [Adafruit Ultimate GPS FeatherWing](https://www.adafruit.com/product/3133) | $39.95 |
| IMU | [Adafruit LSM6DS33 + LIS3MDL - 9 DoF IMU](https://www.adafruit.com/product/4485) | $9.95 |
| Sensor | [Adafruit DPS310 Precision Barometric Pressure Sensor](https://www.adafruit.com/product/4494) | $6.95 |
| Voltage Regulator | [Pololu D36V6F3 3.3V 600mA Step-Down Voltage Regulator](https://www.pololu.com/product/3791) | $4.95 |
| Voltage Regulator | [Pololu D36V6F5 5V 600mA Step-Down Voltage Regulator](https://www.pololu.com/product/3792) | $4.95 |
| Resistor | [2 M 1% 0.6 W resistor](https://www.mouser.ca/ProductDetail/594-MBB02070C2004FCT) | $0.29 |
| Resistor | [1 M 1% 0.6 W resistor](https://www.mouser.ca/ProductDetail/594-B0207C1M000F5T) | $0.20 |
| Capacitor | [0.1 uF Capacitor](https://www.mouser.ca/ProductDetail/Vishay-BC-Components/K104K15X7RF53L2?qs=mWFvmKOfYW8KbAXlf9eSQA%3D%3D) | $0.29 |
| Power Connector |	[Phoenix Contact MSTB 2,5/ 2-ST-5,08 - 1757019](https://www.mouser.ca/ProductDetail/Phoenix-Contact/1757242?qs=%2Fha2pyFadugVjodGKkrF4xNq%252BZEVHysqCHlL2cTnJ%252B8%3D) | $0.67 |
| Power Connector	| [Phoenix Contact MSTBA 2,5/ 2-G-5,08 - 1757242](https://www.mouser.ca/ProductDetail/Phoenix-Contact/1757019?qs=sGAEpiMZZMvlX3nhDDO4AGmxTE5dWGQY3FmaBdjJUN0%3D) | $2.08 |
| Connector |	[Molex PicoBlade PCB Header](https://www.mouser.ca/datasheet/2/276/0530471010_PCB_HEADERS-171035.pdf) | $0.50 |
| Enclosure | [Hammond Manufacturing Polycarbonate 1554WA2GY ](https://www.hammfg.com/part/1554WA2GY) | $30.00 |
| Battery | [Tadiran TLP93121](https://www.tadiranbat.com/assets/tlp-93121-b-al1.pdf) | $150.00 |
| **Total** | | |

#### Photos

<p align="left"><img src="https://user-images.githubusercontent.com/22924092/128431908-5bdc1f6b-d60b-4290-8024-060dc5c09b14.jpeg" width="480"></p>
<p align="left"><b>Figure 2:</b> Assembled Cryologger iceberg drift tracking beacon.</p>

### 2.3 Measurements

**Table 2.** Variables recorded and transmitted by the Cryologger iceberg drift tracking beacon.
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
| Transmit status | | Iridium error return code |
| Iteration counter |  | Number of program iterations |

## 3.0 Deployments

A total of 10 Cryologger iceberg drift tracking beacons were deployed during the 2021 Amundsen Expedition between August 21st to September 3rd, 2021. Deployments were made by helicopter from the CCGS Amundsen on icebergs and ice islands along the coasts of Ellesmere Island, Baffin Island and Greenland (Figure 3). The overall suitability of potential targets was determined by assessing the iceberg’s size, shape, and location. Where possible, icebergs selected to be instrumented with a tracking beacon were large enough to survive drifting south to the Grand Banks of Newfoundland and far enough away from the coast to increase the chances of being carried southward by the currents and avoid becoming grounded in shallow coastal areas.

<p align="center"><img width="720" src="https://user-images.githubusercontent.com/22924092/133437548-ac2ced2f-60ad-4eab-820c-647c70e01970.png"></p>
<p align="left"><b>Figure 3:</b> Map of Cryologger iceberg drift tracking beacons deployed during the 2021 Amundsen Expedition.</p>


<p align="center"><img width="360" src="https://github.com/adamgarbo/Cryologger_Iceberg_Tracking_Beacon/blob/main/Images/2021_300434065869240.JPG">
<img width="360" src="https://github.com/adamgarbo/Cryologger_Iceberg_Tracking_Beacon/blob/main/Images/2021_300434063497310.JPG"></p>
<p align="center"><img width="360" src="https://github.com/adamgarbo/Cryologger_Iceberg_Tracking_Beacon/blob/main/Images/2021_300434063291950.JPG">
<img width="360" src="https://github.com/adamgarbo/Cryologger_Iceberg_Tracking_Beacon/blob/main/Images/2021_300434065864290.JPG"></p>
<p align="left"><b>Figure 4:</b> Examples of icebergs instrumented with Cryologger iceberg tracking beacons during the 2021 Amundsen Expedition.</p>

## 4.0 Results


## 5.0 Conclusion

The Cryologger iceberg tracking beacon harnesses low-cost, open-source hardware and software to provide a robust, cost-effective and user-friendly platform for the collection of long-term iceberg observations. 

The overall success of the Cryologger deployments has demonstrated that it can provide a robust platform for the collection of in-situ iceberg tracking data. With a total cost of under $600 USD in materials each, this represents a cost-effective alternative to existing proprietary commercial systems.

## Repository Contents

* **/Archive** - Contains information on previous versions of the Cryologger design

* **/Software** - Contains the Arduino code

* **/Bill of Materials** - Information on all components used construction and their associated costs

* **/Documentation** - All project documents

* **/Hardware** - KiCad schematics and Fusion 360 design files

## Documentation
* Currently in progress

## License Information
This project is distributed under the GNU General Public License v3.0

Cheers,

**Adam**
