import rospy
import serial
import utm
import time
from Customgps.msg import Float64
#from sensor_msgs.msg import NavSatFix

# Function to check if GPGGA sentence is present
def isGPGGAinString(inputString):
    return '$GPGGA' in inputString

# Function to split the GPGGA string into its components
def splitGPGGA(gpggaString):
    return gpggaString.split(",")

# Function to convert Degrees and Minutes to Decimal Degrees (DD.dddd)
def degMinstoDegDec(LatOrLong):
    if len(LatOrLong) == 9:  # Latitude (format: DDMM.MMMM)
        deg = int(LatOrLong[:2])
        mins = float(LatOrLong[2:])
    else:  # Longitude (format: DDDMM.MMMM)
        deg = int(LatOrLong[:3])
        mins = float(LatOrLong[3:])
    degDec = mins / 60.0
    return deg + degDec

# Function to apply the sign convention for Latitude and Longitude
def LatLongSignConvetion(LatOrLong, LatOrLongDir):
    if LatOrLongDir in ["S", "W"]:
        LatOrLong = -LatOrLong
    return LatOrLong

# Function to convert Latitude and Longitude to UTM
def convertToUTM(LatitudeSigned, LongitudeSigned):
    UTMVals = utm.from_latlon(LatitudeSigned, LongitudeSigned)
    UTMEasting = UTMVals[0]
    UTMNorthing = UTMVals[1]
    UTMZone = UTMVals[2]
    UTMLetter = UTMVals[3]
    return [UTMEasting, UTMNorthing, UTMZone, UTMLetter]

# Function to convert UTC time to ROS format (Epoch time)
def UTCtoROS(UTC):
    # Example: UTC = 123519 -> 12:35:19 UTC (HHMMSS)
    hh = int(UTC[:2])
    mm = int(UTC[2:4])
    ss = float(UTC[4:])
    
    # Get the current date
    current_time = time.gmtime(time.time())
    time_sec = time.mktime((current_time.tm_year, current_time.tm_mon, current_time.tm_mday, hh, mm, int(ss), 0, 0, 0))
    
    # Seconds and nanoseconds for ROS
    current_time_sec = int(time_sec)
    current_time_nsec = int((time_sec - current_time_sec) * 1e9)
    
    return current_time_sec, current_time_nsec

# Function to read from the serial port
def ReadFromSerial(serialPort):
    line = serialPort.readline().decode('ascii').strip()  # Read a line from the serial port
    return line

# Main function
if __name__ == '__main__':
    rospy.init_node('gps_node')

    # Parameters
    serial_port = rospy.get_param('~port', '/dev/ttyUSB0')
    serial_baud = rospy.get_param('~baudrate', 4800)
    gps_topic = rospy.get_param('~gps_topic', '/gps/fix')

    # Open serial port for GPS data
    port = serial.Serial(serial_port, serial_baud, timeout=5.0)

    # ROS publishers
    gps_pub = rospy.Publisher(gps_topic, NavSatFix, queue_size=5)
    utm_pub = rospy.Publisher('/gps/utm', Float64, queue_size=5)

    rospy.loginfo("GPS node started, reading from serial port: " + serial_port)

    try:
        while not rospy.is_shutdown():
            gpggaRead = ReadFromSerial(port)
            
            # Check if the string contains a GPGGA sentence
            if isGPGGAinString(gpggaRead):
                # Split the GPGGA string into components
                gpggaSplit = splitGPGGA(gpggaRead)

                # Extract fields: UTC, Latitude, Longitude, and Directions
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
                UTMEasting, UTMNorthing, UTMZone, UTMLetter = convertToUTM(LatitudeSigned, LongitudeSigned)


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

                rospy.loginfo(f"Published: {custom_gps_msg}")

            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        port.close()
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down GPS driver...")
        port.close()