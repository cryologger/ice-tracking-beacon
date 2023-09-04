# Cryologger Ice Tracking Beacon (ITB) - Pre-Deployment
This guide provides step-by-step instructions on preparing the Cryologger ITBs for deployment.

## System Description:

The Cryologger ITB consists of a water-resistant Nanuk case, which contains the battery and electronics. The electronics are housed in a 3D-printed enclosure. 

<p align="center"><img width="720" src="https://github.com/adamgarbo/cryologger-ice-tracking-beacon/blob/main/Images/cryologger-itb-1.jpeg"></p>

## Pre-Deployment Checklist

### Install Software
* Ensure all steps were followed in INSTALLATION.md to install the necessary Arduino IDE software and related libraries.

### Unpack Cryologger ITBs
- Inspect the packages for any potential damage to the Nanuk enclosures, 

### Assemble Battery
The Cryologger ITB uses a 7.2 V 38 Ah lithium thionyl chloride (Li-SoCl2) battery. The battery is not terminated from the factory and requires a Phoenix Contact PCB connector to be attached to the red and black wires.
Please note: This battery has an extremely high energy density and care must be given to avoid shorting the black and red wires.

- Strip approximately 7 mm from the end of the red and black wires
- Insert each wire into the Phoenix Contact PCB connector one at a time
- Tighten the screw of
- Ensure that no bare wire can be seen sticking out of the PCB connector

### Collect Documentation
It is important to properly document the operation of each Cryologger in order to better troubleshoot issues in the future.

- Launch the Arduino IDE
- Connect to the Cryologger using a mini-USB cable
- Open the Serial Monitor in Arduino IDE
- Observe the serial output as the Cryologger initalizes its sensors and attempts an initial transmission
- Once the transmission is complete, copy all of the serial output to a .txt file 
  - Use the format: `cryologger_itb_2023_X.txt`, where X is the unit number of the Cryologger
- Note the presenece of any errors encountered and report them as soon as possible

### Perform Long-duration Test
It is highly desirable to test the Cryologger ITBs for as long as possible prior to deployment to ensure proper operation. 

Place all Cryologger ITB systems in an area with a good view of the sky.
- It is important that the ITBs are securely fastened to the deck so they do not roll or blow away. This can be accomplished using a rope to lash all Nanuk enclosures together, and then tied off to the deck of the ship.

## Troubleshooting



