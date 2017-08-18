#!/usr/bin/env python
#
# Code created/modified by Andrew Bolduc for the use with Sparkfun IMU SEN-14001 
# Originally adapted from imu_node.py by Tang Tiong Yew 
# Copyright (c) 2012, Tang Tiong Yew All rights reserved.
#
import string
import math
import sys
import rospy
import serial

#from time import time
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
from tf.transformations import quaternion_from_euler
# from dynamic_reconfigure.server import Server
degrees2rad = math.pi/180.0
rospy.init_node("razor_node")
pub = rospy.Publisher('imu_data', Imu, queue_size=1)
pub2 = rospy.Publisher('vel', Point, queue_size=1)
pub3 = rospy.Publisher('raw_imu', Imu, queue_size=1)
r = rospy.Rate(100)


velMsg = Point()
imuMsg = Imu()
raw_imu = Imu()

accel_factor = 9.806
seq = 0.0
samplePeriod = 0
velx = 0.0
vely = 0.0
velz = 0.0
last_velx = 0.0
last_vely = 0.0
last_velz = 0.0
posx = 0.0
posy = 0.0
posz = 0.0
last_posx = 0.0
last_posy = 0.0
last_posz = 0.0
last_time = 0.0
last_a_x=0
last_a_y=0
last_a_z = 0
gravity=[0,0,-9.81]
rotatedGravity = [0,0,0]
motionAcceleration =[0,0,0]
q = [0,0,0,0]

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
    #exit
    sys.exit(0)

rospy.loginfo("Giving the razor IMU board 5 seconds to boot...")
rospy.sleep(5)

rospy.loginfo("Calibrating")
for x in range(0, 100):
    line = ser.readline()
    line = line.replace("#YPRAG=","")   # Delete "#YPRAG="
    words = string.split(line,",")    # Fields split

    imuMsg.orientation.x = float(words[10])
    imuMsg.orientation.y = float(words[11])
    imuMsg.orientation.z = float(words[12])
    imuMsg.orientation.w = float(words[13])

    q[0]=float(words[10])
    q[1]=float(words[11])
    q[2]=float(words[12])
    q[3]=float(words[13])

    acc_x1 = 2*(q[1]*q[3] - q[0]*q[2])
    acc_y1 = 2*(q[0]*q[1]+q[2]*q[3])
    acc_z1 = q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3]

    acc_x = float(words[1])
    acc_y = float(words[2]) 
    acc_z = float(words[3])

    motionAcceleration[0]=(acc_x-acc_x1)*accel_factor
    motionAcceleration[1]=(acc_y-acc_y1)*accel_factor
    motionAcceleration[2]=(acc_z-acc_z1)*accel_factor

    if x > 1:
        last_a_x = (last_a_x*(x-1)+motionAcceleration[0])/x
        last_a_y = (last_a_y*(x-1)+motionAcceleration[1])/x
        last_a_z = (last_a_z*(x-1)+motionAcceleration[2])/x
           
        
rospy.loginfo(last_a_x)
rospy.loginfo(last_a_y)
rospy.loginfo(last_a_z)



rospy.loginfo("Publishing IMU data... /imu /vel /traj")

while not rospy.is_shutdown():
    line = ser.readline()
    line = line.replace("#YPRAG=","")   # Delete "#YPRAG="
    words = string.split(line,",")    # Fields split
    if len(words) > 2:

        # acc_x = float(words[1])*accel_factor 
        # acc_y = float(words[2])*accel_factor 
        # acc_z = -float(words[3])*accel_factor 
        # Angle of rotation

        roll= float(words[14])*degrees2rad
        pitch=  float(words[15])*degrees2rad
        yaw = float(words[16])*degrees2rad
        # rospy.loginfo("%s %s %s", words[14],words[15],words[16] )
        
        imuMsg.angular_velocity.x = float(words[4])
        imuMsg.angular_velocity.y = float(words[5]) 
        imuMsg.angular_velocity.z = float(words[6]) 
        
        # q = quaternion_from_euler(roll,pitch,yaw)
        # imuMsg.orientation.x = q[0]
        # imuMsg.orientation.y = q[1]
        # imuMsg.orientation.z = q[2]
        # imuMsg.orientation.w = q[3]

        imuMsg.orientation.x = float(words[10])
        imuMsg.orientation.y = float(words[11])
        imuMsg.orientation.z = float(words[12])
        imuMsg.orientation.w = float(words[13])

        q[0]=float(words[10])
        q[1]=float(words[11])
        q[2]=float(words[12])
        q[3]=float(words[13])

        acc_x1 = 2*(q[1]*q[3] - q[0]*q[2])
        acc_y1 = 2*(q[0]*q[1]+q[2]*q[3])
        acc_z1 = q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3]

        acc_x = float(words[1])
        acc_y = float(words[2]) 
        acc_z = float(words[3])
        if math.fabs((acc_x-acc_x1)*accel_factor)<math.fabs(last_a_x):
            motionAcceleration[0] = 0
        else:
            motionAcceleration[0]=(acc_x-acc_x1)*accel_factor-last_a_x
        if math.fabs((acc_y-acc_y1)*accel_factor)<math.fabs(last_a_y):
            motionAcceleration[1]=0
        else:
            motionAcceleration[1]=(acc_y-acc_y1)*accel_factor-last_a_y
        if math.fabs((acc_z-acc_z1)*accel_factor)<math.fabs(last_a_z):
            motionAcceleration[2]=0
        else:
            motionAcceleration[2]=(acc_z-acc_z1)*accel_factor-last_a_x

        if seq < 2:
            time_offset = float(words[0])
            # rospy.loginfo(time_offset)
        elif seq > 1:
            samplePeriod = (float(words[0])-time_offset)/1000
            time_offset = float(words[0])
            velx = last_velx + round(motionAcceleration[0],1)*samplePeriod
            vely = last_vely + round(motionAcceleration[1],1)*samplePeriod
            velz = last_velz + round(motionAcceleration[2],1)*samplePeriod
            last_velx = velx
            last_vely = vely
            last_velz = velz
            velMsg.x = last_velx
            velMsg.y = last_vely
            velMsg.z = last_velz
            # rospy.loginfo(last_velx)



        imuMsg.linear_acceleration.x = round(motionAcceleration[0],1)
        imuMsg.linear_acceleration.y = round(motionAcceleration[1],1)
        imuMsg.linear_acceleration.z = round(motionAcceleration[2],1)
 	
	raw_imu.linear_acceleration.x = float(words[1])
        raw_imu.linear_acceleration.y = float(words[2]) 
        raw_imu.linear_acceleration.z = float(words[3])

        # Publish message
    imuMsg.header.stamp= rospy.Time.now()
    imuMsg.header.frame_id = 'base_imu_link'
    imuMsg.header.seq = seq
    seq = seq + 1
    pub.publish(imuMsg)
    pub2.publish(velMsg)
    pub3.publish(raw_imu)
    r.sleep()

ser.close
