#!/usr/bin/env python

"""

"""
import math
import random
import time
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy


class F110_autonomous:
    ''' '''
    def __init__(self):
        # publishers and subscribers
        self.velocity_publisher = rospy.Publisher(
            '/cmd_vel', Twist, queue_size=5)
        self.laser_subscriber = rospy.Subscriber(
            "/scan", LaserScan, self.laser_callback, queue_size=5)
        self.F110_laser = LaserScan()
        # subscribed to joystick inputs on topic "joy"
        rospy.Subscriber("joy", Joy, self.joy_callback)

        # from wanderer
        self.sect_1 = 0
        self.sect_2 = 0
        self.sect_3 = 0
        # self.ang = {0:0,001:-1.2,10:-1.2,11:-1.2,100:1.5,101:1.0,110:1.0,111:1.2}
        self.ang = {0:0,001:-0.2,10:-0.2,11:-0.4,100:0.2,101:0.4,110:0.4,111:0.4} 
        self.fwd = {0:.25,1:0,10:0,11:0,100:0.1,101:0,110:0,111:0}

        # area of distances in the three sectors
        self.sect_left = 0    
        self.sect_center = 0  
        self.sect_right = 0  
        
        # init parameters for steering controller    
        self.KpS = 1        
        self.KdS = 1.2    
        self.last_errorS = 0.0
        self.d_errorS = 0.0       

        # control frequency
        self.control_frequency = 50

        #joyData data from joystic
	self.joyData1 = 0
	self.joyData2 = 0
	self.joybut0 = 0
        
    # reset of sectors
    def reset_sect(self):
        self.sect_1 = 0
        self.sect_2 = 0
        self.sect_3 = 0
        self.sect_center = 0

    # sort the sectors for the control laws
    def sort(self, laserscan):

        entries = len(laserscan.ranges)
        self.sect_left = laserscan.range_max            
        self.sect_center = laserscan.range_max         
        self.sect_right = laserscan.range_max         
        
        for entry in range(0,entries):
            # if 0.4 < laserscan.ranges[entry] < 0.75: # original laser scan ranges
            if 0 < laserscan.ranges[entry] < 0.35:
                self.sect_1 = 1 if (0 < entry < entries/3) else 0 
                self.sect_2 = 1 if (entries/3 < entry < entries/2) else 0
                self.sect_3 = 1 if (entries/2 < entry < entries) else 0

            # identify the area for each sector
            # right sector                   
            if (entries*3/10 < entry < entries*4/10):
                if (laserscan.ranges[entry] != float('inf')):
                    self.sect_right += laserscan.ranges[entry]*0.1

            # center sector
            if (entries*29/60 < entry < entries*31/60):
                if (laserscan.ranges[entry] < self.sect_center):
                    self.sect_center = laserscan.ranges[entry]

            # left sector
            if (entries*6/10 < entry < entries*7/10):
                if (laserscan.ranges[entry] != float('inf')):
                    self.sect_left += laserscan.ranges[entry]*0.1

        #rospy.loginfo("sort complete,sect_1: " + str(self.sect_1) + " sect_2: " + str(self.sect_2) + " sect_3: " + str(self.sect_3) + " sect_center: " + str(self.sect_center))

    def movement(self):
        '''Uses the information known about the obstacles to move robot.'''
        sect = int(str(self.sect_1) + str(self.sect_2) + str(self.sect_3))

        # Calculate error for PD
        errorST = - self.sect_left + self.sect_right

        # dead band
        if (math.fabs(errorST) > 0.1 ): 
            errorS = errorST
        else:
            errorS = 0            
        
        # error calculation
        self.d_errorS = (errorS - self.last_errorS) / self.control_frequency
        self.last_errorS = errorS         

        # Define vel_msg
        vel_msg = Twist()

        if (sect == 0):
            control = self.KpS * errorS + self.KdS * self.d_errorS
        else:
            control = self.ang[sect]

	#saturate the control
	control=np.clip(control, -30, 30)

	#forward control
	FWcontrol = 0
	FWcontrolMaxSpeed = 22 #max speed
	FWcontrolProp = 1.5 # threshold distance 
	FWcontrolStop = 0.5 # threshold critical distance
	if (self.sect_center > (FWcontrolProp+FWcontrolStop)):
		FWcontrol = FWcontrolMaxSpeed
	elif (self.sect_center <= (FWcontrolProp+FWcontrolStop)):
		FWcontrol = (self.sect_center-FWcontrolStop)*FWcontrolMaxSpeed/FWcontrolProp

	# debug
        rospy.loginfo("Sect = " + str(sect) + " - Error: " + str(errorS) + " - Control = " + str(control) + " - Speed = " + str(FWcontrol) + " - frontdist = " + str(self.sect_center))       

        # vel_msg.linear.x = FWcontrol #30*self.joyData1
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        #vel_msg.angular.z = np.clip(control, -30, 30) #30*self.joyData2

	#safety override using the joystic
	if self.joybut0 == 1:
		vel_msg.linear.x = FWcontrol #30*self.joyData1
		vel_msg.angular.z = np.clip(control, -30, 30) #30*self.joyData2
	else:
		vel_msg.linear.x = 0
		vel_msg.angular.z = 0	
	 
        self.velocity_publisher.publish(vel_msg)

	rospy.loginfo("x = " + str(30*self.joyData1) + " - z: " + str(-30*self.joyData2))

        self.reset_sect()
   
    
    def laser_callback(self, laserscan):
        '''Callback function for laser scan '''

        self.sort(laserscan)
        self.movement()

    def joy_callback(self,data):

        self.joyData1 = data.axes[1]
        self.joyData2 = data.axes[2]
	self.joybut0 = data.buttons[0]

    def constrain(self, val, min_val, max_val):
        ''' Constrain val between min and max'''
        return min(max_val, max(min_val, val))
    
def main():

    # Start
    rospy.loginfo('Task start.')
    rospy.loginfo('Init...')

    turtlebot = F110_autonomous()
    rospy.init_node('F110_racing_autonomous', anonymous=True)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
