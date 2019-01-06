'''
Title:  SBD Payload Decoding - Cryologger Tracking Beacons
Author: Adam Garbo 
Date:   January 4, 2019

'''

def decode_cryologger(imei, sbd_raw):
    '''Function to decode Cryologger beacons'''
    import binascii
    import logging
    import sqlite3
    import struct
    # Configure logger
    logging.basicConfig(filename='debug.log',
                        level=logging.DEBUG, 
                        format='%(asctime)s %(message)s')

    # Raw data example containing 3 SBD messages
    #sbd_raw = 'a0312e5cebf529272a063a027e0af664ee03274a4afc08007c00b11a0600ec0cb03f2e5ce5f52c2713061002810a4c67ee03f94a4afc0a005b00401b0000ed0cc04d2e5ca6f531270406d401870aef67ee03f0494afc0c004b001c1b0000ee0c'

    # Display binary data as tuples of hex-values
    sbd_hex = binascii.unhexlify(sbd_raw)

    # Specify SBD message length
    sbd_length = 32 

    # Split into individual messages
    sbd_list = [sbd_hex[i:i+sbd_length] for i in range(0, len(sbd_hex), sbd_length)]

    # Connect to database
    db = sqlite3.connect('isbd.db')
    logging.info('Connected to SQLite database')
    print('Connected to SQLite database')

    # Get a cursor object
    c = db.cursor()

    # Unpack messages and store to database
    for x in sbd_list:

        # Format characters
        sbd_format = '<LhhhhHllHHHHH'

        # Unpack data
        sbd_data = list(struct.unpack(sbd_format,x))

        # Revert variables
        sbd_data[0]             # Unixtime
        sbd_data[1] /= 100.0    # Temperature
        sbd_data[2] /= 100.0    # Pressure
        sbd_data[3] /= 100.0    # Pitch
        sbd_data[4] /= 100.0    # Roll
        sbd_data[5] /= 10.0     # Heading
        sbd_data[6] /= 1000000  # Latitude
        sbd_data[7] /= 1000000  # Longitude
        sbd_data[8]             # Satellites 
        sbd_data[9] /= 100.0    # HDOP
        sbd_data[10] /= 1000.0  # Voltage
        sbd_data[11]            # Transmit time
        sbd_data[12]            # Iteration counter

        # Insert a row of data
        c.execute("INSERT INTO cryologger VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?);", sbd_data)
        logging.info('Inserted: %s' % sbd_data)
        print('Inserted: %s' % sbd_data)
        # Commit the change
        db.commit()

    # Close the db connection
    db.close()
    logging.info('Database connection closed')

