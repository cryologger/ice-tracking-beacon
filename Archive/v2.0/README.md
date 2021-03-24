# Cryologger - Iceberg Tracking Beacon (ITB) v2.0
![Image](https://github.com/adamgarbo/Cryologger_Iceberg_Tracking_Beacon/blob/main/Archive/v2.0/Images/2019_300434063392070.JPG)

## Background
Icebergs and ice islands represent significant hazards to marine navigation and offshore infrastructure at a time when demand for access to Canada’s Arctic waters is increasing.

There is a growing demand for in situ iceberg tracking data to monitor their drift trajectory and improve predictions of ice hazard occurrence and behaviour, yet the high cost of commercial tracking devices often prevents monitoring at optimal spatial and temporal resolutions.

## Objective
The goal of this project is to determine if the Cryologger, a tracking beacon based on inexpensive, open-source hardware and software, can provide a reliable and cost-effective platform for monitoring the drift of icebergs and ice islands in the Canadian Arctic.

## Design
The Cryologger is based on the open-source Arduino platform (www.arduino.cc) and built using low-cost, do-it-yourself electronics that can be easily modified to meet the needs of the end-user. Code was written using the Arduino Integrated Development Environment (IDE) and benefits from the availability of community-generated libraries.

Planned for extended deployments in harsh Arctic conditions, the Cryologger can provide long-term measurements of GPS position, temperature, pressure, pitch, roll, tilt-compensated heading and battery voltage. 

### Bill of Materials

**Table 1.** Bill of materials
| Component | Product | Cost (USD) | URL |
| --- | --- | :---: | --- |
| Satellite transceiver | Rock7 RockBLOCK 9603 | $249.95 | http://www.rock7mobile.com/products-rockblock-9603 |
| Processor | Adafruit Feather M0 Basic Proto | $19.95 | https://www.adafruit.com/product/2772 |
| Real-time clock | Adafruit DS3231 Precision RTC FeatherWing | $13.95 | https://www.adafruit.com/product/3028 |
| Real-time clock battery | CR1220 12mm Diameter - 3V Lithium Coin Cell Battery | $2.50 | https://www.adafruit.com/product/380 |
| GPS | Adafruit Ultimate GPS FeatherWing | $39.95 | https://www.adafruit.com/product/3133 |
| GPS battery | CR1220 12mm Diameter - 3V Lithium Coin Cell Battery | $2.50 | https://www.adafruit.com/product/380 |
| IMU | Pololu LSM303D 3D Compass and Accelerometer | $7.95 | https://www.pololu.com/product/2127 |
| Voltage Regulator | Pololu D36V6F3 3.3V 600mA Step-Down Voltage Regulator | $4.95 | https://www.pololu.com/product/3791 |
| Power connector | Male DC Power adapter - 2.1mm plug to screw terminal block | $2.00 | https://www.adafruit.com/product/369 |
| Power connector | Breadboard-friendly 2.1mm DC barrel jack | $0.95 | https://www.adafruit.com/product/373 |
| Protoboard | Adafruit FeatherWing Tripler Mini Kit | $8.50 | https://www.adafruit.com/product/3417 |
| Battery |	Tadiran	Lithium Pulses Plus 7.2V 38Ah 273.6Wh | $100 | http://www.tadiranbat.com/assets/tlp-93121-b-al1.pdf |
| Enclosure | Nanuk 903 | $27.95 | https://nanuk.com/products/nanuk-903 |
| **Total** | | **$481.10** | |

### Assembly

<p align="center"><img src="https://github.com/adamgarbo/Cryologger_Iceberg_Tracking_Beacon/blob/main/Archive/v2.0/Images/assembly_1.JPG" width="720"></p>
<p align="center"><b>Figure 1.</b> Assembly of Cryologger beacons prior to deployment.</p>
<p align="center"><img src="https://github.com/adamgarbo/Cryologger_Iceberg_Tracking_Beacon/blob/main/Archive/v2.0/Images/assembly_2.JPG" width="720"></p>
<p align="center"><b>Figure 2.</b> Affixing of wooden spike beds to Cryologger enclosures.</p>
<p align="center"><img src="https://github.com/adamgarbo/Cryologger_Iceberg_Tracking_Beacon/blob/main/Archive/v2.0/Images/assembly_3.JPG" width="720"></p>
<p align="center"><b>Figure 3.</b> Testing of Iridium transceivers from atop the CCGS Amundsen.</p>

### Measurements

**Table 2.** Collected measurements
| Variable | Unit | Comments | Size |
| --- | :---: | --- | --- |
| Datetime  |   | UNIX Epoch time | 4 bytes |
| Temperature | °C  | Internal temperature | 2 bytes |
| Pitch | °|  | 2 bytes |
| Roll | ° |  | 2 bytes |
| Heading | °  | Tilt-compensated heading (0-360°) | 2 bytes |
| Latitude | DD |  | 4 bytes |
| Longitude | DD |  | 4 bytes |
| Satellites | # | Number of satellites in view  | 2 bytes |
| HDOP |  | Horizonal dilution of precision | 2 bytes |
| Voltage | V | Battery voltage | 2 bytes |
| Transmit duration  | s | Length of Iridium transmission  | 2 bytes |
| Message counter |  | Number of transmitted messages | 4 bytes |
| | | | **Total: 30 bytes** |

### Data Transmission and Processing
Sensor measurements and GPS position are recorded hourly and stored in memory until the desired transmission interval is reached. Data are compressed into a binary message (340 bytes maximum) to minimize the cost and total number of transmissions required. Data are transmitted via the Iridium Short Burst Data (SBD) satellite network at user-specified intervals, which can be remotely updated based on the desired sampling frequency. SBD data messages are received by an Iridium ground station and sent to Rock7's server. The data is then forwarded to an Amazon Web Services (AWS) SQS queue, decoded using an AWS Lambda Python function and stored in a database using the Amazon Relational Database Service (RDS). 

Data is made freely available and can be viewed in near-real time at https://cryologger.org.

## Deployments

A total of 8 Cryologger drift tracking beacons were deployed from the CCGS Amundsen's helicopter on icebergs and ice islands along the coasts of Ellesmere Island, Baffin Island and Greenland during the ArcticNet leg of the 2019 Amundsen Expedition. An additional 2 tracking beacons were also deployed by helicopter on icebergs near the Milne Ice Shelf, Ellesmere Island in the summer of 2019.

<p align="center"><img src="https://github.com/adamgarbo/Cryologger_Iceberg_Tracking_Beacon/blob/main/Archive/v2.0/Images/2019_deployments.png" width="720"></p>
<p align="center"><b>Figure 3.</b> 2019 Cryologger deployment locations.<p>

The overall suitability of potential targets was determined by assessing the iceberg’s size, shape and location. At the time of deployment, a compass heading of the tracking beacon was recorded and a 360° aerial photo survey of the iceberg was performed if possible. Following deployment, communications with each beacon were successfully established in order to remotely modify their operational parameters for optimal battery efficiency.

<p align="center">
  <img src="https://github.com/adamgarbo/Cryologger_Iceberg_Tracking_Beacon/blob/main/Archive/v2.0/Images/2019_300434063494100_A.JPG" width="420">
  <img src="https://github.com/adamgarbo/Cryologger_Iceberg_Tracking_Beacon/blob/main/Archive/v2.0/Images/2019_300434063494100_B.JPG" width="420">
</p>
<p align="center"><b>Figure 3.</b> Deployment of a Cryologger beacon on an iceberg approximately 700 m in length.<p>

## Results

Tracking beacons deployed in the summer of 2019 have reported a combined total of over 40,000 GPS positions as of December 2020. Several beacons have experienced difficulties transmissitting data due to the use of the RockBLOCK's onboard antenna and interference from the overlying snow cover.

**Table 3.** Cryologger v2.0 deployments and days operational as of December 22, 2020.
| Beacon	| Deployment	| Latitude	| Longitude	| Days Operational | Comments
| :---: | :---: | :---: | :---: | :---: | --- | 
| 65700	| 2019-07-06 17:00	| 82.7173	| -82.7923	| 535 | Petermann Glacier deployment |
| 54740	| 2019-07-08 01:00	| 82.7588	| -82.1242	| 534 | Petermann Glacier deployment |
| 96100	| 2019-08-01 12:50	| 80.9552	| -61.6162	| 479 | Petermann Glacier deployment |
| 92350	| 2019-08-01 13:00	| 80.9693	| -61.1920	| 399 | Petermann Glacier deployment |
| 92950	| 2019-08-01 13:15	| 80.9083	| -60.9950	| 433 | |
| 98160	| 2019-08-01 13:30	| 80.8669	| -61.3386	| 480 | |
| 94100	| 2019-07-30 14:00	| 79.3992	| -65.1887	| 424 | |
| 92070	| 2019-07-30 15:00	| 79.7245	| -65.3903	| 413 | |
| 94110	| 2019-08-05 15:15	| 77.9722	| -78.4142	| 331 | Iceberg broke apart approximately July 1, 2020 |
| 95310	| 2019-08-05 16:15	| 77.9225	| -78.1355	| 431 | |

## Conclusion & Future Work

The overall success of the Cryologger tracking beacon deployments have demonstrated that low-cost, open-source hardware and software can provide a robust and cost-effective platform for the collection in-situ iceberg tracking data. These data can provide key insights to iceberg drift and deterioration processes in the Canadian Arctic.

Development of the next version of the Cryologger tracking beacon is currently underway and will focus on improving overall reliability, exploring the use of more environmentally friendly materials and modifications allowing the tracking beacons to transform into ocean drifter buoys once breakup of the icebergs occurs.

## Repository Contents

* **/Software** - Contains the Arduino code.

* **/Bill of Materials** - Information on all components used construction and their associated costs.

* **/Documentation** - All project documentation.

* **/Hardware** - Autodesk EAGLE schematics and Fusion 360 design files.

## Documentation
* Currently in progress.

## License Information
This project is distributed under the GNU General Public License v3.0.

Cheers,

**Adam**
