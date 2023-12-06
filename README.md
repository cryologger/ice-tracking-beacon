# Cryologger - Iceberg Tracking Beacon (ITB)
<p align="left">
<img alt="GitHub" src="https://img.shields.io/github/license/adamgarbo/Cryologger_Iceberg_Tracking_Beacon">
<img alt="GitHub release (latest by date)" src="https://img.shields.io/github/v/release/adamgarbo/Cryologger_Iceberg_Tracking_Beacon">
<img alt="GitHub issues" src="https://img.shields.io/github/issues/adamgarbo/Cryologger_Iceberg_Tracking_Beacon">
</p>

### Note: This project is under active development. 

![2021_300434065734810](https://user-images.githubusercontent.com/22924092/158026368-e336462b-1e3f-46a9-8846-9ab7fdb8b459.JPG)

## 1.0 Introduction
Icebergs and ice islands represent significant hazards to marine navigation and offshore infrastructure at a time when demand for access to Canada’s Arctic waters is increasing. There is a growing demand for in situ iceberg tracking data to monitor their drift trajectory and improve predictions of ice hazard occurrence and behaviour, yet the high cost of commercial tracking devices often prevents monitoring at optimal spatial and temporal resolutions.

The Cryologger Ice Tracking Beacon (ITB), built using inexpensive, open-source hardware and software, can provide a reliable and cost-effective platform for monitoring the drift of icebergs and ice islands in the Canadian Arctic.

## 2.0 Methods

### 2.1 Design

The Cryologger ITB is based on the open-source Arduino platform (www.arduino.cc) and built using low-cost, do-it-yourself electronics that can be easily modified to meet the needs of the end-user. Code was written using the Arduino Integrated Development Environment (IDE) and benefits from the availability of community-generated libraries.

Planned for extended deployments in harsh Arctic conditions, the Cryologger ITB provides long-term measurements of GNSS position, temperature, pressure, pitch, roll, tilt-compensated heading and battery voltage. Data are transmitted over the Iridium satellite network at specified intervals and can be remotely updated based on the desired sampling frequency. Collected data are made available in near-real time at https://cryologger.org.

#### 2.1.1 Design iterations

Version 1.0

Version 2.0 

Version 3.0 of the Cryologger ITB builds on the successes of v1.0 and v2.0 and is also based on the Adafruit ecosystem of components. A number of improvements to the design were made, including:
* 3.3 V power is now provided directly from a Pololu 3.3 V step-down voltage regulator, bypassing the Feather M0's onboard AP2112 LDO regulator (-55 μA).
* A dedicated 5 V step-down voltage regulator was added to power the RockBLOCK Iridium transceiver.
* The DS3231 real-time clock (RTC) was removed in favour of the SAMD21's internal RTC for all timekeeping and alarm functionality with periodic time synchronizations with the GPS.
* The LSM303 accelerometer/magnetometer was replaced with the LSM6DS33 + LIS3MDL IMU due to the sensor reaching its end-of-life (EOL).
* The temperature/pressure and IMU sensors are now powered directly through SAMD21 GPIO pins, allowing for power to be completely removed during sleep.
* A 10MΩ + 1 MΩ resistor divider is now used to measured the battery voltage (+2.4 μA).

#### 2.1.2 Custom PCB
One of the major change in v3.0 is a custom carrier board PCB that is designed to greatly simplify the assembly process. The PCB was designed in KiCad and fabricated by JLCPCB. 

<p align="center"><img src="https://user-images.githubusercontent.com/22924092/222990941-2d50d191-8055-475c-9464-4d9a1b2d12a2.png" width="720"></p>  
<p align="center"><b>Figure 1:</b> 3D rendering of Cryologger iceberg drift tracking beacon carrier board designed in KiCad.</p>

### 2.2 Bill of Materials 

**Table 1.** Bill of materials and associated costs for components used in the Cryologger ITB v3.1. Prices are listed in USD and are current as of December 2023. Taxes and shipping not included. <sup>1</sup>Denotes optional component. Please note bill of materials is a work in progress.
| Component | Product | Quantity | Cost (USD) |
| --- | --- | :---: | :---: |
| PCB | [Custom Cryologger Printed Circuit Board](https://jlcpcb.com) | 1 | $5.00 | 
| Satellite transceiver | [Rock7 RockBLOCK 9603](https://www.groundcontrol.com/us/product/rockblock-9603-compact-plug-and-play-satellite-transmitter/) | 1 | $267.50 |
| Satellite antenna<sup>1</sup> | [Maxtena M1621HCT-P-SMA](https://maxtena.com/products/f-passive/m1621hct-p-sma-iridium-passive-antenna/) | 1 | $54.00 |
| Microcontroller | [Adafruit Feather M0 Basic Proto](https://www.adafruit.com/product/2772) | 1 | $19.95 |
| GNSS Receiver | [Adafruit Ultimate GPS FeatherWing](https://www.adafruit.com/product/3133) | 1 | $24.95 |
| IMU | [Adafruit LSM303AGR Accelerometer Magnetometer](https://www.adafruit.com/product/4413) | 1 | $12.50 |
| Sensor | [Adafruit BME280 Temperature Humidity Pressure Sensor](https://www.adafruit.com/product/2652) | 1 | $14.95 |
| Voltage regulator | [Pololu D36V6F3 3.3V 600mA Step-Down Voltage Regulator](https://www.pololu.com/product/3791) | 1 | $6.95 |
| Voltage regulator | [Pololu D36V6F5 5V 600mA Step-Down Voltage Regulator](https://www.pololu.com/product/3792) | 1 | $6.95 |
| Resistor | [10 M 1% 0.6 W resistor](https://www.mouser.ca/ProductDetail/594-MBB02070C1005FCT) | 1 | $0.29 |
| Resistor | [1 M 1% 0.6 W resistor](https://www.mouser.ca/ProductDetail/594-B0207C1M000F5T) | 1 | $0.20 |
| Capacitor | [0.1 uF Capacitor](https://mou.sr/481ILov) | 1 | $0.23 |
| Power connector |	[Phoenix Contact MSTB 2,5/ 2-ST-5,08 - 1757019](https://mou.sr/47HAA0B) | 1 | $0.60 |
| Power connector	| [Phoenix Contact MSTBA 2,5/ 2-G-5,08 - 1757242](https://mou.sr/3R27Ick) | 1 | $1.81 |
| Connector |	[Molex PicoBlade PCB Header](https://mou.sr/3qLrmgc) | 1| $0.55 |
| Connector |	[Molex PicoBlade Crimp Housing(https://mou.sr/3lXOY2x])| 2| $0.36 |
| Cable Assembly |	Molex PicoBlade Pre-crimped Jumper Wire 2" Yellow | 1 | |
| Cable Assembly |	Molex PicoBlade Pre-crimped Jumper Wire 2" Orange | 1 | |
| Cable Assembly |	Molex PicoBlade Pre-crimped Jumper Wire 2" Black | 1 | |
| Cable Assembly |	Molex PicoBlade Pre-crimped Jumper Wire 2" Slate | 1 | |
| Cable Assembly |	Molex PicoBlade Pre-crimped Jumper Wire 2" Red | 1 | |
| Enclosure | [Nanuk 904](https://nanukcases.ca/products/nanuk-904) | 1 | $47.95 |
| Battery | [Tadiran TLP93121](https://www.tadiranbat.com/assets/tlp-93121-b-al1.pdf) | 1 | $145.50 |
| **Total** | | | |

#### Photos

<p align="center"><img src="https://github.com/adamgarbo/cryologger-ice-tracking-beacon/blob/main/Images/cryologger-itb-2.jpeg" width="480" ></p>
<p align="center"><b>Figure 2:</b> Assembled Cryologger ice drift tracking beacon housed in Nanuk case and 3D-printed case lid removed.</p>

### 2.3 Measurements

**Table 2.** Variables recorded and transmitted by the Cryologger iceberg drift tracking beacon.
| Variable | Unit | Comments |
| --- | :---: | --- |
| Datetime  |   | YYYY-MM-DD HH:MM:SS |
| Temperature | °C  | Internal temperature |
| Humidity | %  | Internal humidity |
| Pressure | hPa | Internal pressure |
| Pitch | °|  |
| Roll | ° |  |
| Heading | °  | Tilt-compensated heading (0-360°) |
| Latitude | DD |  |
| Longitude | DD |  |
| Satellites | # | Number of satellites in view  |
| HDOP |  | Horizonal dilution of precision |
| Voltage | V | Battery voltage |
| Transmit duration  | s | Length of Iridium transmission  |
| Transmit status | | Iridium error return code |
| Iteration counter |  | Number of program iterations |

## 3.0 Deployments

A total of 37 Cryologger iceberg drift tracking beacons have been deployed between 2018 and 2023 (Figure X), primarily during the annual Amundsen Expedition. Deployments are performed by helicopter from the CCGS Amundsen on icebergs and ice islands along the coasts of Ellesmere Island, Baffin Island and Greenland (Figure 3). The overall suitability of potential targets was determined by assessing the iceberg’s size, shape, and location. Where possible, icebergs selected to be instrumented with a tracking beacon were large enough to survive drifting south to the Grand Banks of Newfoundland and far enough away from the coast to increase the chances of being carried southward by the currents and avoid becoming grounded in shallow coastal areas. 

A case study of deployments from the 2021 Amundsen Expedition is presented below in 3.1. 

<p align="center"><img width="480" src="https://github.com/cryologger/ice-tracking-beacon/blob/main/Images/cryologger-itb-deployments.png"></p>
<p align="left"><b>Figure 3:</b> Map of Cryologger ice drift tracking beacons deployed between 2018 and 2023.</p>

### 3.1 2021 Amundsen Expedition



<p align="center"><img width="640" src="https://user-images.githubusercontent.com/22924092/133437548-ac2ced2f-60ad-4eab-820c-647c70e01970.png"></p>

<p align="center"><img src="https://github.com/adamgarbo/Cryologger_Iceberg_Tracking_Beacon/blob/main/Images/2021_300434065869240.JPG" width="360" >
<img width="360" src="https://github.com/adamgarbo/Cryologger_Iceberg_Tracking_Beacon/blob/main/Images/2021_300434063497310.JPG"></p>
<p align="center"><img width="360" src="https://github.com/cryologger/ice-tracking-beacon/blob/main/Images/2021_300434063291950.JPG">
<img width="360" src="https://github.com/adamgarbo/Cryologger_Iceberg_Tracking_Beacon/blob/main/Images/2021_300434065864290.JPG"></p>
<p align="left"><b>Figure 4:</b> Examples of icebergs instrumented with Cryologger iceberg tracking beacons during the 2021 Amundsen Expedition.</p>

## 4.0 Results


As of December 2023, Cryologger ITBs deployments made on icebergs and ice islands have achieved up to 1530 days of continuous operation. Collectively, these ITBs transmitted more than 125,000 GNSS positions and travelled a combined distance of over 12,000 km. The recorded iceberg drift tracks of all Cryologger ITB deployments can be viewed at: https://cryologger.org/tracking

### 4.1 2021 Amundsen Expedition



## 5.0 Conclusion

The Cryologger iceberg tracking beacon harnesses low-cost, open-source hardware and software to provide a robust, cost-effective and user-friendly platform for the collection of long-term iceberg observations. 

The overall success of the Cryologger deployments has demonstrated that it can provide a robust platform for the collection of in-situ iceberg tracking data. With a total cost of of approximately $700 USD in materials each, this represents a cost-effective alternative to existing proprietary commercial systems.

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
