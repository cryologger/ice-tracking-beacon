# Cryologger - Iceberg Tracking Beacon (ITB)
<p align="left">
<img alt="GitHub" src="https://img.shields.io/github/license/adamgarbo/Cryologger_Iceberg_Tracking_Beacon">
<img alt="GitHub release (latest by date)" src="https://img.shields.io/github/v/release/adamgarbo/Cryologger_Iceberg_Tracking_Beacon">
<img alt="GitHub issues" src="https://img.shields.io/github/issues/adamgarbo/Cryologger_Iceberg_Tracking_Beacon">
<img alt="DOI" src="https://zenodo.org/badge/163880339.svg">
</p>

### Note: This project is under active development. 

![2021_300434065734810](https://user-images.githubusercontent.com/22924092/158026368-e336462b-1e3f-46a9-8846-9ab7fdb8b459.JPG)

## 1.0 Introduction
Icebergs and ice islands represent significant hazards to marine navigation and offshore infrastructure at a time when demand for access to Canada’s Arctic waters is increasing. There is a growing demand for in situ iceberg tracking data to monitor their drift trajectory and improve predictions of ice hazard occurrence and behaviour, yet the high cost of commercial tracking devices often prevents monitoring at optimal spatial and temporal resolutions.

The Cryologger Ice Tracking Beacon (ITB), built using inexpensive, open-source hardware and software, provides a reliable and cost-effective platform for monitoring the drift of icebergs and ice islands in the Canadian Arctic.

## 2.0 Methods

### 2.1 Design

The Cryologger ITB is based on the open-source Arduino platform (www.arduino.cc) and built using low-cost, do-it-yourself electronics that can be easily modified to meet the needs of the end-user. Code was written using the Arduino Integrated Development Environment (IDE) and benefits from the availability of community-generated libraries.

Planned for extended deployments in harsh Arctic conditions, the Cryologger ITB provides long-term measurements of GNSS position, temperature, pressure, pitch, roll, tilt-compensated heading and battery voltage. Data are transmitted over the Iridium satellite network at specified intervals and can be remotely updated based on the desired sampling frequency. Collected data are made available in near-real time at https://cryologger.org.

#### 2.1.1 Design Iterations

More information about the design of versions 1.0 and 2.0 of the Cryologger ITB can be found in the following publication:

[Garbo, A., & Mueller, D. (2024). Cryologger Ice Tracking Beacon: A Low-Cost, Open-Source Platform for Tracking Icebergs and Ice Islands. _Sensors_, _24_(4), Article 4. https://doi.org/10.3390/s24041044](https://www.mdpi.com/1424-8220/24/4/1044)

Version 3.0 of the Cryologger ITB builds on the successes of previous versions and features a number of improvements to the design were made, including:
* 3.3 V power is now provided directly from a Pololu 3.3 V step-down voltage regulator, bypassing the Feather M0's onboard AP2112 LDO regulator (-55 μA).
* A dedicated 5 V step-down voltage regulator was added to power the RockBLOCK Iridium transceiver.
* The DS3231 real-time clock (RTC) was removed in favour of the SAMD21's internal RTC for all timekeeping and alarm functionality with periodic time synchronizations with the GPS.
* The LSM303 accelerometer/magnetometer was replaced with the LSM6DS33 + LIS3MDL IMU due to the sensor reaching its end-of-life (EOL).
* The temperature/pressure and IMU sensors are now powered directly through SAMD21 GPIO pins, allowing for power to be completely removed during sleep.
* A 10MΩ + 1 MΩ resistor divider is now used to measured the battery voltage (+2.4 μA).

A major change in v3.0 is a custom carrier board PCB that is designed to greatly simplify the assembly process. The PCB was designed in KiCad and fabricated by JLCPCB. 

<p align="center"><img src="https://user-images.githubusercontent.com/22924092/222990941-2d50d191-8055-475c-9464-4d9a1b2d12a2.png" width="720"></p>  
<p align="center"><b>Figure 1:</b> 3D rendering of Cryologger iceberg drift tracking beacon carrier board designed in KiCad.</p>

#### 2.1.2 Bill of Materials (BOM)

**Table 1.** Bill of materials and associated costs for components used in the Cryologger ITB v3.1. Prices are listed in USD and are current as of December 2023. Taxes and shipping not included. <sup>1</sup>Denotes optional component. Please note BOM is a work in progress.
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
| Connector |	[Molex PicoBlade Crimp Housing](https://mou.sr/3lXOY2x])| 2| $0.36 |
| Cable Assembly |	Molex PicoBlade Pre-crimped Jumper Wires | 5 | |
| Enclosure | [Nanuk 904](https://nanukcases.ca/products/nanuk-904) | 1 | $47.95 |
| Battery | [Tadiran TLP93121](https://www.tadiranbat.com/assets/tlp-93121-b-al1.pdf) | 1 | $145.50 |
| **Total** | | | |

<p align="center"><img src="https://github.com/adamgarbo/cryologger-ice-tracking-beacon/blob/main/Images/cryologger-itb-2.jpeg" width="480" ></p>
<p align="center"><b>Figure 2:</b> Assembled Cryologger ice drift tracking beacon housed in Nanuk case and 3D-printed case lid removed.</p>

### 2.2 Operation

The programming logic of the Cryologger ITB optimizes sleep and wake cycles to minimize overall power consumption. When initially powered on, the system attempts to acquire a signal from the GNSS receiver and synchronizes the RTC with the current date and time. It then sets an alarm for the initial sampling interval and enters a low-power, deep-sleep mode. When the alarm triggers, the system wakes, records the time, and acquires a GNSS position. Next, measurements of all onboard sensors are collected (Table 2). These data are stored into memory and when the software determines the appropriate transmission interval has been met, it attempts to transmit the data using the RockBLOCK satellite transceiver. The software then disables power to all components, sets the next alarm, and returns to sleep.

#### 2.2.1 Measurements

**Table 2.** Measurements recorded and transmitted by the Cryologger ITB v3.0, including variable sizes.
|     Variable             |     Unit    |     Description                                   |     Size (bytes)    |
|--------------------------|-------------|---------------------------------------------------|---------------------|
|     unixtime             |     s       |     Unix time (seconds since 1970-01-01)          |     4               |
|     temperature_int      |     °C      |     Internal temperature                          |     2               |
|     humidity_int         |     %       |     Internal humidity                             |     2               |   
|     pressure_int         |     hPa     |     Internal pressure                             |     2               |   
|     pitch                |     °       |     Pitch angle                                   |     2               |   
|     roll                 |     °       |     Roll angle                                    |     2               |   
|     heading              |     °       |     Tilt-compensated magnetic heading (0-360°)    |     2               |   
|     latitude             |     °       |     GNSS latitude                                 |     4               |   
|     longitude            |     °       |     GNSS longitude                                |     4               |   
|     satellites           |             |     Number of GNSS satellites in view             |     2               |   
|     hdop                 |             |     GNSS horizontal dilution of precision         |     2               |   
|     voltage              |     V       |     Battery voltage                               |     2               |   
|     transmit_duration    |     s       |     Transmission time of SBD message              |     2               |   
|     transmit_status      |             |     Iridium return code                           |     1               |   
|     message_counter      |             |     Number of transmitted messages                |     2               |   

#### 2.2.2 Data Transmission and Processing

The Cryologger ITB records its position and sensor measurements nominally on an hourly basis and transmits at an interval of 3 hours. Data are transmitted via the Iridium satellite network as a Short Burst Data (SBD) message. The ITB attempts to transmit each message for up to 180 seconds, and if unsuccessful, the message is stored in a temporary buffer and reattempted at the next transmission interval. Both the sampling and transmission frequency of individual Cryologgers can be remotely modified by the end-user. Successfully transmitted SBD messages are received by an Iridium ground station and sent to Ground Control's server. These data are then forwarded to Amazon Web Services (AWS), where they are decoded using a Python script, stored in a database, and visualized on the Cryologger website: https://cryologger.org.

## 3.0 Deployments

A total of 37 Cryologger ITBs have been deployed between 2018 and 2023 (Figure 3), primarily during the annual Amundsen Expedition. Deployments are performed by helicopter from the CCGS Amundsen on icebergs and ice islands along the coasts of Ellesmere Island, Baffin Island and Greenland (Figure 3). At the time of deployment, a compass heading of the tracking beacon was recorded. Where possible, a 360° aerial photo survey of the iceberg was performed for use with Structure-from-Motion photogrammetry in order to create 3D models of the iceberg.

<p align="center"><img width="480" src="https://github.com/cryologger/ice-tracking-beacon/blob/main/Images/cryologger-itb-deployments.png"></p>
<p align="left"><b>Figure 3:</b> Map of Cryologger ice drift tracking beacons deployed between 2018 and 2023.</p>

The overall suitability of potential targets was determined by assessing the iceberg’s size, shape, and location. Where possible, icebergs selected to be instrumented with a tracking beacon were large enough to survive drifting south to the Grand Banks of Newfoundland and far enough away from the coast to increase the chances of being carried southward by the currents and avoid becoming grounded in shallow coastal areas. 

<p align="center"><img src="https://github.com/cryologger/ice-tracking-beacon/blob/main/Images/2021_300434065869240.JPG" width="360" >
<img width="360" src="https://github.com/cryologger/ice-tracking-beacon/blob/main/Images/2021_300434063497310.JPG"></p>
<p align="center"><img width="360" src="https://github.com/cryologger/ice-tracking-beacon/blob/main/Images/2021_300434063291950.JPG">
<img width="360" src="https://github.com/cryologger/ice-tracking-beacon/blob/main/Images/2021_300434065864290.JPG"></p>
<p align="center"><b>Figure 4:</b> Examples of icebergs instrumented with Cryologger iceberg tracking beacons during the 2021 Amundsen Expedition.</p>

## 4.0 Results

The operation of the Cryologger ITBs greatly exceeded expectations, with the operational lifespans of deployments made in 2018 ranging from 310 to 1530 days, and those in 2019 ranging from 333 to 615 days. The most frequent causes of loss of communication with the ITBs was due to the eventual deterioration (calving or rolling) or catastrophic break-up of the iceberg and subsequent destruction of the beacon. In many cases, it was possible to use remote sensing imagery to observe major break-up events of icebergs that immediately resulted in transmissions from the beacon to cease.

### 4.1 Iceberg Drift
A snapshot of iceberg drift tracks recorded from all Cryologger ITB deployments as of December 2023 is shown in Figure 4. This data can also be viewed in real-time at: https://cryologger.org/tracking

<p align="center"><img width="480" src="https://github.com/cryologger/ice-tracking-beacon/assets/22924092/225039bd-b640-42a6-af0c-1c30aa13ba80"></p>
<p align="center"><b>Figure 5:</b> Map of Cryologger iceberg drift track tracks from beacons deployed between 2018 and 2023.</p>

## 5.0 Conclusion

The overall success of the Cryologger ITB has demonstrated that it can provide a robust platform for the collection of in-situ iceberg tracking data. With a total cost of of approximately $700 USD in materials each, this represents a cost-effective alternative to existing proprietary commercial systems. 

The development and deployment of these novel iceberg tracking beacons was intended to enhance and supplement existing iceberg observation networks within the Canadian Arctic and data collected from the Cryologgers has already contributed to a database of iceberg tracking beacon tracks compiled by the Canadian Ice Service (CIS). Most importantly, this research demonstrates to the scientific community that inexpensive, open-source hardware and software can provide a viable solution for the monitoring of icebergs and ice island drift patterns within the Canadian Arctic and beyond!

Development of the next iteration of the Cryologger ITB is currently underway, which is focused on improving overall reliability, exploring the use of environmentally friendly materials, and improvements that will allow the it to transform into an ocean drifter buoys once breakup of the icebergs occurs. 

## Repository Contents

* **/Documentation** - Assembly, deployment and troubleshoting guides, as well as information of components used and associated costs.

* **/Hardware** - KiCad PCB schematic and design files.

* **/Software** -  Arduino code and Python data analysis scripts.

## License Information
This project is released under the GNU General Public License v3.0 (https://www.gnu.org/licenses/gpl-3.0.en.html).

Cheers,

**Adam Garbo**
