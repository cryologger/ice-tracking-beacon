# Cryologger Ice Tracking Beacon (ITB) - Pre-Deployment
This guide provides step-by-step instructions on preparing the Cryologger ITBs for deployment.

## System Description:

The Cryologger ITB consists of a water-resistant Nanuk case, which contains the battery and electronics. The electronics are housed in a 3D-printed enclosure. 

<p align="center"><img width="720" src="https://github.com/adamgarbo/cryologger-ice-tracking-beacon/blob/main/Images/cryologger-itb-1.jpeg"></p>

## Pre-Deployment Checklist

### Install Software
* Ensure all steps were followed in [INSTALLATION.md](https://github.com/adamgarbo/cryologger-ice-tracking-beacon/blob/main/Documentation/INSTALLATION.md) to install the necessary Arduino IDE software and related libraries.

### Unpack Equipment
- Inspect the Cryologger ITBs for any potential damage to the Nanuk enclosures, electronics and batteries.

### Assemble Battery
The Cryologger ITB uses a 7.2 V 38 Ah lithium thionyl chloride (Li-SoCl2) battery. The battery is not terminated from the factory and requires a Phoenix Contact PCB connector to be attached.  
**Warning:** This battery has an extremely high energy density and care must be given to avoid shorting the black and red wires.

- Begin by stripping approximately 7 mm from the end of the red (+) and black (-) wires
- Insert each wire into the Phoenix Contact PCB connector one at a time
- Tighten the screw using sufficient force to ensure the wire cannot be pulled out of the connector
- Ensure no bare wire can be seen sticking out of the PCB connector
- Triple check that the red (+/right) and black (-/left) wires are in the correct positions before plugging in the battery.
  - The PCB is also marked with -/+ and can be used to confirm the correct orientation.
<p align="center"><img width="720" src="https://github.com/adamgarbo/cryologger-ice-tracking-beacon/blob/main/Images/cryologger-itb-battery-1.jpeg"></p>

### Assemble Wooden Platform



## Testing

### Initial Test & Documentation
The first step is conduct a test of each Cryologger ITB to ensure it can properly transmit a message. It is advantageous to document the operation of each Cryologger at the same time in order to better troubleshoot any issues that may be encoutnered.

- Launch the Arduino IDE
- Connect to the ITB using a mini-USB cable
- Connect the battery to the ITB
- Open the Serial Monitor in Arduino IDE
- Observe the serial output as the Cryologger initalizes its sensors and attempts an initial transmission
- Once the transmission is complete, copy the complete the serial output to a .txt file 
  - Use the format: `cryologger_itb_2023_X.txt`, where X is the unit number of the Cryologger
- Also Note the presenece of any errors encountered and report them as soon as possible

### Long-Duration Test
Next, a long-duration test of all of the Cryologger ITBs can be conducted. This is an important step to ensure their proper operation prior to deployment. The tests should be run for a minimum of 24 hours and ideally can continue for up to 1 week. 
- Place the ITBs in an area with a good view of the sky
- Secure the ITBs they do not roll or blow away using a rope to lash all Nanuk enclosures together, and then tied off to the deck of the ship
<p align="center">
  <img width="360" src="https://github.com/adamgarbo/cryologger-ice-tracking-beacon/blob/main/Images/cryologger-itb-test-1.jpeg">
  <img width="360" src="https://github.com/adamgarbo/cryologger-ice-tracking-beacon/blob/main/Images/cryologger-itb-test-2.jpeg">
</p>

- Power on each ITB one at a time and observe the LED blink patterns
- Document the time each Cryologger is turned on.
- Review the data on the Cryologger website: https://cryologger.org/amundsen-2023

## Troubleshooting
Document any issues that arise so that they can be addressed as needed.
