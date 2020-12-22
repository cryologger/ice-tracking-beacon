# Cryologger - Iceberg Tracking Beacon (ITB) v1.0
![Image](https://github.com/adamgarbo/Cryologger_Iceberg_Tracking_Beacon/blob/master/Archive/V1/Photos/IMG_1654.JPG)


## Background
Icebergs and ice islands represent significant hazards to marine navigation and offshore infrastructure at a time when demand for access to Canada’s Arctic waters is increasing.

There is a growing demand for in situ iceberg tracking data to monitor their drift trajectory and improve predictions of ice hazard occurrence and behaviour, yet the high cost of commercial tracking devices often prevents monitoring at optimal spatial and temporal resolutions.

## Objective
The goal of this project is to determine if the Cryologger, a tracking beacon based on inexpensive, open-source hardware and software, can provide a reliable and cost-effective platform for monitoring the drift of icebergs and ice islands in the Canadian Arctic.

## Design
The Cryologger is based on the open-source Arduino platform (www.arduino.cc) and built using low-cost, do-it-yourself electronics that can be easily modified to meet the needs of the end-user. Code was written using the Arduino Integrated Development Environment (IDE) and benefits from the availability of community-generated libraries.

Planned for extended deployments in harsh Arctic conditions, the Cryologger can provide long-term measurements of GPS position, temperature, pressure, pitch, roll, tilt-compensated heading and battery voltage. Data are transmitted over the Iridium satellite network at specified intervals and can be remotely updated based on the desired sampling frequency. Collected data are made freely available and can be viewed in near-real time at https://cryologger.org.

#### Bill of Materials

| Component | Product | Cost (USD) | URL |
| --- | --- | :---: | --- |
| Satellite transceiver | Rock7 RockBLOCK 9603 | $249.95 | http://www.rock7mobile.com/products-rockblock-9603 |
| Satellite antenna | Maxtena	M1621HCT-P-SMA | $50.00 | https://www.maxtena.com/products/f-passive/m1621hct-p-sma-iridium-passive-antenna/ |
| Processor | Adafruit	Pro Trinket - 3V 12MHz | $9.95 | https://www.adafruit.com/products/2010 |
| GNSS | Adafruit	Ultimate GPS Breakout | $39.95 | https://www.adafruit.com/products/746 |
| IMU | Adafruit LSM303 - Triple-axis Accelerometer + Magnetometer | $14.95 | https://www.adafruit.com/products/1120 |
| Real-time clock IC |	Maxim	DS3231SN	1	|	$9.00 |	https://www.digikey.ca/product-detail/en/maxim-integrated/DS3231SN-T-R/DS3231SN-T-RCT-ND/3894827
| Real-time clock PCB	| Adafruit SMT Breakout PCB for SOIC-16 | $3.95	|	https://www.adafruit.com/product/1207
| Real-time clock battery	| Adafruit	20mm Coin Cell Breakout Board (CR2032)	|	$2.50	| https://www.adafruit.com/product/1870
| EEPROM | Microchip	256K I2C EEPROM 24LC256 | $1.25 | https://www.digikey.ca/en/products/detail/microchip-technology/24LC256-I-P/273431 |
| Sensors | Adafruit MPL3115A2 Pressure/Altitude/Temperature Sensor | $9.95 | https://www.adafruit.com/product/1893 |
| Voltage Regulator | 5V, 2.5A Step-Down Voltage Regulator D24V22F5 | $8.95 | https://www.pololu.com/product/2858 |
| Power connector | Male DC Power adapter - 2.1mm plug to screw terminal block | $2.00 | https://www.adafruit.com/product/369 |
| Power connector | Breadboard-friendly 2.1mm DC barrel jack | $0.95 | https://www.adafruit.com/product/373 |
| Protoboard | Adafruit	Perma-Proto Full-sized Breadboard PCB | $5.95	| https://www.adafruit.com/product/1606 |
| Battery | Tadiran Lithium Pulses Plus 7.2V 38Ah 273.6Wh | $100.00 | http://www.tadiranbat.com/assets/tlp-93121-b-al1.pdf |
| Enclosure | Nanuk 905 | $45.95 | https://nanuk.com/collections/all-nanuk-cases/products/nanuk-905 |
| **Total** | | **$555.25** | 

#### Collected Measurements
| Variable | Unit | Comments |
| --- | :---: | --- |
| Datetime  |   | UNIX Epoch Time |
| Temperature | °C  | Internal temperature |
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
| Message counter |  | Number of transmitted messages |

#### Photos
<p float="middle">
<img src="https://github.com/adamgarbo/Cryologger_Iceberg_Tracking_Beacon/blob/master/Archive/V1/Photos/IMG_4288.JPG" width="360">
<img src="https://github.com/adamgarbo/Cryologger_Iceberg_Tracking_Beacon/blob/master/Archive/V1/Photos/IMG_0153.JPG" width="360">
</p>
  
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
