#!/usr/bin/env python

import rospy
import serial
import utm
import time
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix  # Replace with your package name

# Function to check for GPGGA in the string
def isGPGGAinString(inputString):
    return '$GPGGA' in inputString

# Function to split the GPGGA string
def splitGPGGA(gpggaString):
    return gpggaString.split(",")

# Function to convert Degrees and Minutes to Decimal Degrees
def degMinstoDegDec(LatOrLong):
    if len(LatOrLong) == 9:  # Latitude
        deg = int(LatOrLong[:2])
        mins = float(LatOrLong[2:])
    else:  # Longitude
        deg = int(LatOrLong[:3])
        mins = float(LatOrLong[3:])
    return deg + mins / 60.0

# Function to apply sign convention
def LatLongSignConvetion(LatOrLong, LatOrLongDir):
    if LatOrLongDir in ["S", "W"]:
        return -LatOrLong
    return LatOrLong

# Function to convert latitude and longitude to UTM
def convertToUTM(LatitudeSigned, LongitudeSigned):
    UTMVals = utm.from_latlon(LatitudeSigned, LongitudeSigned)
    return UTMVals[0], UTMVals[1], UTMVals[2], UTMVals[3]

# Function to convert UTC time to ROS format (Epoch time)
def UTCtoEpoch(UTC):
    hh = int(UTC[:2])
    mm = int(UTC[2:4])
    ss = float(UTC[4:])
    
    # Get current date for the epoch conversion
    current_time = time.gmtime(time.time())
    time_sec = time.mktime((current_time.tm_year, current_time.tm_mon, current_time.tm_mday, hh, mm, int(ss), 0, 0, 0))
    
    return int(time_sec), int((time_sec - int(time_sec)) * 1e9)

# Function to read from the serial port
def ReadFromSerial(serialPort):
    return serialPort.readline().decode('ascii').strip()

# Main function
if __name__ == '__main__':
    rospy.init_node('gps_driver')

    # Parameters
    serial_port = rospy.get_param('~port', '/dev/ttyUSB0')
    serial_baud = rospy.get_param('~baudrate', 4800)
    gps_topic = '/gps/custom'

    # Open serial port for GPS data
    port = serial.Serial(serial_port, serial_baud, timeout=3.0)

    # ROS publisher
    gps_pub = rospy.Publisher(gps_topic, Float64, queue_size=5)

    rospy.loginfo("GPS driver started, reading from serial port: " + serial_port)

    try:
        while not rospy.is_shutdown():
            gpggaRead = ReadFromSerial(port)
            
            if isGPGGAinString(gpggaRead):
                gpggaSplit = splitGPGGA(gpggaRead)

                # Extract fields
                UTC = gpggaSplit[1]
                Latitude = gpggaSplit[2]
                LatitudeDir = gpggaSplit[3]
                Longitude = gpggaSplit[4]
                LongitudeDir = gpggaSplit[5]
                HDOP = float(gpggaSplit[8]) if gpggaSplit[8] else 0.0
                Altitude = float(gpggaSplit[9]) if gpggaSplit[9] else 0.0

                # Convert Latitude and Longitude to Decimal Degrees
                LatitudeDD = degMinstoDegDec(Latitude)
                LongitudeDD = degMinstoDegDec(Longitude)

                # Apply sign conventions
                LatitudeSigned = LatLongSignConvetion(LatitudeDD, LatitudeDir)
                LongitudeSigned = LatLongSignConvetion(LongitudeDD, LongitudeDir)

                # Convert to UTM
                UTM_Easting, UTM_Northing, UTM_Zone, UTM_Letter = convertToUTM(LatitudeSigned, LongitudeSigned)

                # Convert UTC to epoch time
                epoch_sec, epoch_nsec = UTCtoEpoch(UTC)

                # Create Customgps message
                custom_gps_msg = Customgps()
                custom_gps_msg.header = Header()
                custom_gps_msg.header.frame_id = 'GPS1_Frame'
                custom_gps_msg.header.stamp.sec = epoch_sec
                custom_gps_msg.header.stamp.nsec = epoch_nsec
                custom_gps_msg.latitude = LatitudeSigned
                custom_gps_msg.longitude = LongitudeSigned
                custom_gps_msg.altitude = Altitude
                custom_gps_msg.utm_easting = UTM_Easting
                custom_gps_msg.utm_northing = UTM_Northing
                custom_gps_msg.zone = UTM_Zone
                custom_gps_msg.letter = UTM_Letter
                custom_gps_msg.hdop = HDOP
                custom_gps_msg.gpgga_read = gpggaRead

                # Publish the Customgps message
                gps_pub.publish(custom_gps_msg)

                #rospy.loginfo(f"Published: {custom_gps_msg}")

            #except ValueError as e:
                 #rospy.logwarn("Parsing error: " + str(e))

        rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down GPS driver...")
     
