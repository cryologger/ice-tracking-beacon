# Changelog:

November, 2020
 * Added Qwiic Power Switch to control power to components with high current consumption (e.g. u-blox SAM-M8Q, ICM-20948). 
 * The SparkFun Buck-Boost Converter power the 3.3 V bus directly. ~110-141 uA in sleep mode (total) at 7.4 - 12.5 V

November 18, 2020
* Initial prototype tests performed.

December 20, 2020
* The decision was made to abandon the MicroMod-based beacon prototype. While the MicroMod ecosystem has potential, at present the MicroMod Data Logging Carrier Board (MMDLC) coupled with the Artemis Processor presents a number of significant hardware bugs (i.e. ADC, voltage divider) and a considerably high quiescent draw of ~300 uA.

December 28, 2020
* Removed all SparkX components from design (i.e. Qwiic Power Switch/Qwiic Iridium 9603N) due to concern of being able to reliably source components in the future (i.e. no stock, no longer producing).

December 29, 2020
* Added Qwiic RV-8803 RTC to address extreme clock drift experienced by the SparkFun Qwiic Micro due to lack of SAMD21 external RTC crystal.

December 30, 2020
* Added P-MOSFET to control power to perhipherals (e.g. GNSS, IMU, LED).

December 31, 2020
* Prototype tests performed. 
* Overall good results but neither the Buck-Boost or Pololu step-down regulator are capable of powering the RockBLOCK 9603 with 3.3 V on the 3.7 V LiIon pin. 
* Possible solutions include a dedicated 5.0 V regulator.

January 1, 2021
* Changed processing of incoming MT-SBD messages to use union/structure. 
* This will require that all MT messages be sent in little endian byte order.
* Measured quiescent draw of ~55 uA between Buck-Boost and 3.3 V rail in deep sleep.
* Regulator current will add ~60-130 uA depending.
