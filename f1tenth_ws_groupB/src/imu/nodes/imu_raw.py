#!/usr/bin/env python
#
# Code created by Andrew Bolduc
# This code is intended to be used with hector_localization
# For the use with Sparkfun IMU SEN-14001 
#
import string
import math
import sys
import rospy
import serial

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped

rospy.init_node("razor_node")
pub = rospy.Publisher('raw_imu', Imu, queue_size=1)
pub2 = rospy.Publisher('magnetic', Vector3Stamped, queue_size=1)


magMsg = Vector3Stamped()
imuMsg = Imu()

accel_factor = 9.806
seq = 0.0

imuMsg.orientation_covariance = [
0.0025 , 0 , 0,
0, 0.0025, 0,
0, 0, 0.0025
]

imuMsg.angular_velocity_covariance = [
0.02, 0 , 0,
0 , 0.02, 0,
0 , 0 , 0.02
]

imuMsg.linear_acceleration_covariance = [
0.04 , 0 , 0,
0 , 0.04, 0,
0 , 0 , 0.04
]

default_port='/dev/ttyACM0'
port = rospy.get_param('~port', default_port)

# Check COM port and baud rate
rospy.loginfo("Opening %s...", port)
try:
    ser = serial.Serial(port=port, baudrate=115200, timeout=1)
except serial.serialutil.SerialException:
    rospy.logerr("IMU not found at port "+port + ". Did you specify the correct port?")
    sys.exit(0)

rospy.loginfo("Writing serial command 'c'")

ser.write('c')

rospy.loginfo("Publishing IMU data... /raw_imu /magnetic")

while not rospy.is_shutdown():
    line = ser.readline()
    line = line.replace("#YPRAG=","")   # Delete "#YPRAG="
    words = string.split(line,",")      # Fields split
    if len(words) > 2:

        imuMsg.linear_acceleration.x = float(words[1]) 
        imuMsg.linear_acceleration.y = float(words[2]) 
        imuMsg.linear_acceleration.z = float(words[3])

        imuMsg.angular_velocity.x = float(words[4])
        imuMsg.angular_velocity.y = float(words[5]) 
        imuMsg.angular_velocity.z = float(words[6]) 
        
        imuMsg.orientation.x = float(words[10])
        imuMsg.orientation.y = float(words[11])
        imuMsg.orientation.z = float(words[12])
        imuMsg.orientation.w = float(words[13])

        magMsg.vector.x = float(words[7])
        magMsg.vector.y = float(words[8])
        magMsg.vector.z = float(words[9])  

    # Publish message
    imuMsg.header.stamp= rospy.Time.now()
    imuMsg.header.frame_id = 'base_imu_link'
    imuMsg.header.seq = seq
    
    magMsg.header.stamp= rospy.Time.now()
    magMsg.header.frame_id = 'base_imu_link'
    magMsg.header.seq = seq
    
    seq = seq + 1
    pub.publish(imuMsg)
    pub2.publish(magMsg)
ser.close
