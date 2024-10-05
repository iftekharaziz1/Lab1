#!/usr/bin/env python

import rospy
import serial
import utm
import time
from std_msgs.msg import Header
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix  # Replace with your package name


# Function to check if GPGGA is in the string
def isGPGGAinString(inputString):
    return '$GPGGA' in inputString

# Function to process GPGGA string
def processGPGGA(gpggaString):
    return gpggaString.split(",")

# Function to convert Degrees/Minutes to Decimal Degrees and apply sign convention
def convertToDecimal(value, direction):
    if not value:
        raise ValueError("Invalid Lat/Lon value")
    deg = int(value[:2]) if len(value) == 9 else int(value[:3])  # Latitude uses 2 digits for degrees, Longitude uses 3
    mins = float(value[2:]) if len(value) == 9 else float(value[3:])
    decimal = deg + mins / 60.0
    return -decimal if direction in ['S', 'W'] else decimal

# Function to read from serial port
def readFromSerial(serialPort):
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

    rospy.loginfo(f"GPS driver started, reading from: {serial_port}")

    try:
        while not rospy.is_shutdown():
            gpggaRead = readFromSerial(port)

            if isGPGGAinString(gpggaRead):
                gpggaSplit = processGPGGA(gpggaRead)

                # Check if Latitude and Longitude fields are populated
                if len(gpggaSplit) > 5 and gpggaSplit[2] and gpggaSplit[4]:
                    try:
                        # Process Latitude and Longitude
                        latitude = convertToDecimal(gpggaSplit[2], gpggaSplit[3])
                        longitude = convertToDecimal(gpggaSplit[4], gpggaSplit[5])

                        # Convert to UTM
                        utm_easting, utm_northing, utm_zone, utm_letter = utm.from_latlon(latitude, longitude)

                        rospy.loginfo(f"Lat: {latitude}, Lon: {longitude}, UTM: {utm_easting}, {utm_northing}")

                        # Create and publish message (replace with your custom message)
                        gps_pub.publish(Float64(latitude))  # Example of publishing latitude

                    except ValueError as e:
                        rospy.logwarn(f"Error in Lat/Lon conversion: {str(e)}")
                else:
                    rospy.logwarn(f"Invalid GPGGA string: {gpggaRead}")

        rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down GPS driver...")
    except serial.SerialException as e:
        rospy.logerr(f"Serial error: {str(e)}")
    except Exception as e:
        rospy.logerr(f"Error: {str(e)}")
