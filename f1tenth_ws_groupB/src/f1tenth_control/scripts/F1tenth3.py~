#!/usr/bin/env python

'''
Author: Longxiang Guo
Latest Update: 20170502
All rights reserved
'''

import rospy
import math
import numpy as np
import time
import pid
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from keyboard.msg import Key

middle = 12
sect_increment = 7.5
indicis = np.arange(0,6,1)

def chunks(list,n):
    return (list[i:i+n] for i in xrange(0, len(list), n))
def chunk_overlap(list, group_size, overlap_size):
    return (list[i:i+group_size] for i in xrange(0, len(list), group_size-overlap_size))

def closest(list, Number):
    aux = []
    for valor in list:
        aux.append(abs(Number-valor))
    if not aux:
        return 0
    else:
        return aux.index(min(aux))


class Navigate():

    def __init__(self):

        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=50)
        self.depth_subscriber = rospy.Subscriber('/scan', LaserScan, self.DepthCallback, queue_size=50)
        self.teleop_subscriber = rospy.Subscriber('/keyboard/keyup', Key, self.KeyCallback, queue_size=50)
        self.depth = LaserScan()
        self.mindis = np.zeros(25)
        self.minpos = np.zeros(25)
        self.avgdis = np.zeros(25)
        self.avgwidth = np.zeros(25)
        self.absmindis = 3
        self.globalmin = 3
        self.globalmin_x = 3
        self.globalmin_xdis = 3
        self.globalmin_xpos = 0
        self.globalmax = 0.1
        self.globalminpos = 0
        self.globalmaxpos = 0
        self.maxavgpos = 0
        self.offset = 0
        self.offset2 = 0
        self.spd = 0
        self.brake = 0
        self.sidemaxavg = 0
        self.strcontroller = pid.PID(20, 0, 24, -0.1, 0.1)
        self.strlist = []
        self.trackarealeft = 0
        self.trackarearight = 0
    
    def KeyCallback(self,msg):
        keyinput =  msg.code
        if keyinput == 32:
            self.spd = 0
            self.brake = 1
        elif keyinput == 273:
            self.spd+= 1
            self.brake = 0
        elif keyinput == 274:
            self.spd-= 1
            self.brake = 0

    def DepthCallback(self, msg):
        self.depth = msg
        self.Sort()
        self.offsetestimate()

    def Sort(self):

        sectorsize = 120
        overlap = 90
        stepsize = sectorsize - overlap
        ranges_temp = list(self.depth.ranges)
        ranges = ranges_temp[120:960]
        # Chunk the data from rear side
        total = len(ranges)
        x_coord = []
        x_coordindex = []
        x_coord_left = []
        x_coordindex_left = []
        x_coord_right = []
        x_coordindex_right = []
        for i in range(total):
            ranges[i] = min(ranges[i], 3)
            if i < 380:
		ranges_trim = min(ranges[i], 2.5)
                x_pos = abs(ranges_trim*math.sin(i*self.depth.angle_increment-math.radians(105)))
                x_coord_right.append(min(0.2,x_pos))
                x_coordindex_right.append(i)
                x_coord.append(x_pos)
                x_coordindex.append(i)
            elif i > 460:
		ranges_trim = min(ranges[i], 2.5)
                x_pos = abs(ranges_trim*math.sin(i*self.depth.angle_increment-math.radians(105)))
                x_coord_left.append(min(0.2,x_pos))
                x_coordindex_left.append(i)
                x_coord.append(x_pos)
                x_coordindex.append(i)
        self.trackarealeft = sum(x_coord_left) / len(x_coord_left) 
        self.trackarearight = sum(x_coord_right) / len(x_coord_right)
        self.globalmin = min(ranges)
        self.globalmax = max(ranges)
        self.globalmin_x = min(x_coord)
        globalmin_xindex = x_coordindex[x_coord.index(self.globalmin_x)]
        self.globalmin_xdis = ranges[globalmin_xindex]
        self.globalmin_xpos = globalmin_xindex*self.depth.angle_increment - math.radians(105)
        self.globalminpos = ranges.index(self.globalmin)*self.depth.angle_increment + self.depth.angle_min
        self.globalmaxpos = ranges.index(self.globalmax)*self.depth.angle_increment + self.depth.angle_min
        chunk_range_temp = list(chunk_overlap(ranges, sectorsize, overlap))
        chunk_range = chunk_range_temp[0:25]
        for j in range(25):
            minvalue = min(chunk_range[j])
            self.mindis[j] = minvalue
            minindices = chunk_range[j].index(minvalue)

            minpos = self.depth.angle_min + (j*stepsize + minindices + 1) * self.depth.angle_increment
            self.minpos[j] = minpos

            avgvalue = sum(chunk_range[j]) / len(chunk_range[j])
            self.avgdis[j] = avgvalue

            avgwid = avgvalue * math.sin(stepsize*self.depth.angle_increment)
            self.avgwidth[j] = avgwid
            #rospy.loginfo("minvalue: %f, minpos: %f, avgvalue: %f, avgwid: %f", minvalue,minpos, avgvalue,avgwid )

    def offsetestimate(self):
        
        leftavgs = np.flipud(self.avgdis[19:25])
        leftangles = np.flipud(np.radians((19+indicis-middle)*sect_increment))
        rightavgs = self.avgdis[0:6]
        rightangles = np.radians((indicis-middle)*sect_increment)
        leftx = np.multiply(np.sin(leftangles),leftavgs)
        lefty = np.multiply(np.cos(leftangles),leftavgs)
        rightx = np.multiply(np.sin(rightangles),rightavgs)
        righty = np.multiply(np.cos(rightangles),rightavgs)
        middlex = (leftx + rightx)/2
        middley = (lefty + righty)/2
        p = np.poly1d(np.polyfit(middley,middlex,1))
        k = (np.sum(leftavgs)-np.sum(rightavgs))/2		
        self.offset = p(0)
        self.offset2 = k
        self.sidmaxavg = max([max(leftavgs),max(rightavgs)])

    def IntelMove(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.
        vel_msg.linear.y = 0.
        vel_msg.linear.z = 0.
        vel_msg.angular.x = 0.
        vel_msg.angular.y = 0.
        vel_msg.angular.z = 0.
        spd = 0.
        rot = 0.
        speeddead = 0
        speedlimit = 29
        rotatelimit = 35
        emergent_flag = 0

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():

            #Calculate distance indicis
            maxavg = max(self.avgdis)
            maxavgpos = [m for m, n in enumerate(self.avgdis) if n == maxavg]
            maxavgangleindex = closest(maxavgpos,middle)
            if -1< maxavgangleindex < len(maxavgpos) :
                maxavgangle = maxavgpos[maxavgangleindex]
            else:
                maxavgangle = middle
            minavg = min(self.avgdis)
            minavgangle = np.argmin(self.avgdis)

            #Calculate vehicle spd
            if self.brake == 0:
                speeddead= self.spd
                if max(self.avgdis[middle],self.avgdis[middle+1],self.avgdis[middle-1] )> max(1.3,1*self.spd/15): 
                    if self.globalmin > 0.7:
                        spd = speeddead + 6* self.globalmin
                    elif self.globalmin > 0.2:
                        spd = speeddead + 3* self.globalmin
                        rospy.loginfo("Too Close ")
                    spd = min(spd, speedlimit)
                else:
                    spd = 0
            else:
                spd = 0
            #Limit speed range
            if speeddead < 22.1:
                vel_msg.linear.x = min(speeddead+3, spd)
            else:
                vel_msg.linear.x = min(speedlimit, spd)
  
            # Calculate target angle
            target = maxavgangle
            targetangle = math.radians((target - middle) * sect_increment)

	    #Calculate avoid angle
            avoid_x = 0.3
            avoid_r = 2
            if abs(self.globalmin_x) < avoid_x and self.globalmin_xdis < avoid_r:
                track_error = self.trackarealeft - self.trackarearight
                avoidangle = 1.5*math.sqrt(math.sqrt(abs(track_error)))*np.sign(track_error)
		avoidflag = 1
            else:
                avoidangle = 0
		avoidflag = 0

            # Calculate center force
            #if abs(self.offset) > 0.45:
            #if self.mindis[middle] > 1.5:
	    if avoidflag == 0:
		if abs(self.offset) > 0.45:
                   center_force =  2*math.atan(3*self.offset/max(spd,1))
		else:
		   center_force =  math.atan(3*self.offset/max(spd,1))
            else:
                center_force = 0
            
            #Combine 3 components together
            finaltargetgain = max(1.6,1.6*spd/18)
            finaltarget = finaltargetgain*targetangle
            finalcenter = 0*center_force
            finalavoid = 0.8*avoidangle
            finalangle = finaltarget + finalcenter + finalavoid
            #rot = self.strcontroller.update_PID(finalangle)

            rot = 50*finalangle
            
            # Limit steering range
            rot = rot -4 # Neutral compensation
            absrot = abs(rot)
            temprot = min(absrot, rotatelimit)
            
            # Inverse steering control from Gazebo
            if rot > 0:
                str_value = -temprot
            else:
                str_value = temprot
            
            vel_msg.angular.z = str_value
            rospy.loginfo("target: %f , avoid: %f , center: %f, offset: %f, spd: %f , rot: %f ",
            finaltarget, avoidangle,finalcenter,self.offset, vel_msg.linear.x, vel_msg.angular.z)
            self.cmd_vel.publish(vel_msg)
            rate.sleep()

        self.cmd_vel.publish(Twist())
    def shutdown(self):
        rospy.loginfo("Stop F1tenth")
        self.cmd_vel.publish(Twist())
        time.sleep(1)

if __name__ == '__main__':
    try:
        rospy.init_node('F1tenth', anonymous=False)
        navi = Navigate()

        while not rospy.is_shutdown():
            time.sleep(0.1)
            navi.IntelMove()


    except rospy.ROSInterruptException:
        rospy.loginfo("Node Terminated")

