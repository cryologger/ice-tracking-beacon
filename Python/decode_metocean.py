'''
Title:  SBD Payload Decoding - MetOcean PAWS
Author: Adam Garbo 
Date:   January 3, 2019

'''

def decode_metocean(imei, sbd_raw):
    import binascii
    import logging
    import sqlite3

    # The magical function
    def dec(b):
        return int(b,2)

    # Configure logger
    logging.basicConfig(filename='debug.log',
                        level=logging.DEBUG, 
                        format='%(asctime)s %(message)s')

    # Raw data example from MetOcean PAWS manual
    #sbd_raw = '2940d6cf86c577950b0b02c0b909e77afc6e180dd2288a2940202940d6cf86c577950b0b02c0b909e77afc6e180dd2288a294020'


    # Display binary data as tuples of hex-values
    sbd_hex = binascii.unhexlify(sbd_raw)

    # Specify SBD message length
    sbd_length = 26

    # Split into individual messages
    sbd_list = [sbd_hex[i:i+sbd_length] for i in range(0, len(sbd_hex), sbd_length)]

    # Connect to database
    db = sqlite3.connect('isbd.db')
    logging.info('Connected to SQLite database')

    # Get a cursor object
    c = db.cursor()

    # Unpack messages and store to database
    for x in sbd_list:

        # Convert hexadecimal to binary
        sbd_bin = bin(int(sbd_raw, 16))[2:]

        # Determine if binary padding is required
        n_pad = sbd_length * 8 - len(sbd_bin)

        # Pad binary with zeroes
        sbd_bin = '0' * n_pad + sbd_bin

        # Splice up based on position and set to cooresponding variables
        sbd_data = list()
        sbd_data.append(dec(sbd_bin[0:16]) * 0.25)              # JulianHour
        sbd_data.append(dec(sbd_bin[16:27]) * 0.1 + 850)        # BarometricPressure
        sbd_data.append(dec(sbd_bin[27:36]) * 0.1 - 25.5)       # BPTendency
        sbd_data.append(dec(sbd_bin[36:46]) * 0.1 - 60)         # SeaSurfaceTemperature
        sbd_data.append(dec(sbd_bin[46:56]) * 0.1 - 60)         # AirTemperature
        sbd_data.append(dec(sbd_bin[56:64]) * 0.5)              # RelativeHumidity
        sbd_data.append(dec(sbd_bin[64:72]) * 0.25)             # ScalarWindSpeed
        sbd_data.append(dec(sbd_bin[72:80]) * 0.25)             # VectorWindSpeed
        sbd_data.append(dec(sbd_bin[80:90]) * 0.3519)           # WindDirection(Unit Vect)
        sbd_data.append(dec(sbd_bin[90:100]) * 0.3519)          # WindDirection(vect)
        sbd_data.append(dec(sbd_bin[100:110]) * 0.3519)         # CompassHeading
        sbd_data.append(dec(sbd_bin[110:120]) * 0.1 - 50)       # CompassPitch
        sbd_data.append(dec(sbd_bin[120:130]) * 0.1 - 50)       # CompassRoll
        sbd_data.append(dec(sbd_bin[130:150]) * 0.00018 - 90)   # Latitude
        sbd_data.append(dec(sbd_bin[150:171]) * 0.00018 - 180)  # Longitude
        sbd_data.append(dec(sbd_bin[171:177]) * 0.15 + 10.75)   # Voltage
        sbd_data.append(dec(sbd_bin[177:184]) * 3)              # SBDTIME
        sbd_data.append(dec(sbd_bin[184:200]) * 0.25)           # GPSFIXJHOUR
        sbd_data.append(dec(sbd_bin[200:208]))                  # TTFF

        # Insert a row of data
        c.execute("INSERT INTO metocean VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?);", sbd_data)
        logging.info('Inserted row: %s' % sbd_data)
        print('Inserted row: %s' % sbd_data)

        # Commit the change
        db.commit()

    # Close the db connection
    db.close()
    logging.info('Database connection closed')