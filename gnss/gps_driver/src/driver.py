#!/usr/bin/env python

import serial
import rospy
import sys
import utm
from Customgps.msg import gps

serial_connection = serial.Serial('/dev/ttyUSB0')
serial_connection.baudrate = 4800

gps_data = gps()

def publish_gps_data():
    gps_publisher = rospy.Publisher('gps_data', gps, queue_size=10)
    rospy.init_node('gps_publisher_node', anonymous=True)
    loop_rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        raw_data = str(serial_connection.readline())
        data_split = raw_data.split("b'")
        csv_data = data_split[0].split(",")

        if csv_data[0] == "$GPGGA":
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

if __name__ == '__main__':
    publish_gps_data()
