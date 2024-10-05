#!/usr/bin/env python

import serial
import rospy
import sys
import utm
from Customgps.msg import gps

try:
    serial_connection = serial.Serial('/dev/ttyUSB0', baudrate=4800, timeout=1)
except serial.SerialException as e:
    rospy.logerr(f"Serial connection error: {e}")
    sys.exit(1)

gps_data = gps()

def publish_gps_data():
    gps_publisher = rospy.Publisher('gps_data', gps, queue_size=10)
    rospy.init_node('gps_publisher_node', anonymous=True)
    loop_rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            raw_data = serial_connection.readline().decode('ascii', errors='replace').strip()
            if not raw_data.startswith("$GPGGA"):
                continue

            csv_data = raw_data.split(",")
            lat_raw = float(csv_data[2])
            lat_minutes = lat_raw % 100
            lat_degrees = int(lat_raw / 100)
            latitude = lat_degrees + (lat_minutes / 60)

            lon_raw = float(csv_data[4])
            lon_minutes = lon_raw % 100
            lon_degrees = int(lon_raw / 100)
            longitude = lon_degrees + (lon_minutes / 60)
            longitude = -longitude

            utm_coords = utm.from_latlon(latitude, longitude)

            elev = float(csv_data[9])
            hdop_value = float(csv_data[8])
            utc_time = float(csv_data[1])
           
            gps_data.header = csv_data[0]
            gps_data.latitude = latitude
            gps_data.longitude = longitude
            gps_data.altitude = elev
            gps_data.utm_easting = utm_coords[0]
            gps_data.utm_northing = utm_coords[1]
            gps_data.HDOP = hdop_value
            gps_data.UTC = utc_time

            gps_data.zone = utm_coords[2]
            gps_data.letter = utm_coords[3]
            rospy.loginfo(gps_data)
            gps_publisher.publish(gps_data)
            loop_rate.sleep()
        except (ValueError, IndexError) as e:
            rospy.logwarn(f"Data parsing error: {e}")

if __name__ == '__main__':
    publish_gps_data()