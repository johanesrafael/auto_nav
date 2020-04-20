#!/usr/bin/env python2
# -*- coding: utf-8 -*-
# FINAL CODE FOR NAVIGATION AND MAPPING

import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
import math
import cmath
import numpy as np
import time

laser_range = np.array([])
occdata = []
yaw = 0.0
t_yaw=0.0
rotate_speed = 0.20
linear_speed = 0.15
stop_distance = 0.30
occ_bins = [-1, 0, 100, 101]
front_angle = 5
front_angles = range(-front_angle,front_angle+1,1)
side_angles_f = range(43,48,1)
side_angles_b = range(133,138,1)



def get_odom_dir(msg):
    global yaw

    orientation_quat =  msg.pose.pose.orientation
    orientation_list = [orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)


def get_laserscan(msg):
    global laser_range

    # create numpy array
    laser_range = np.array([msg.ranges])


def get_occupancy(msg):
    global occdata

    # create numpy array
    occdata = np.array([msg.data])
    # compute histogram to identify percent of bins with -1
    occ_counts = np.histogram(occdata,occ_bins)
    # calculate total number of bins
    total_bins = msg.info.width * msg.info.height
    # log the info
    #rospy.loginfo('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i', occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins)

def rotatebot(rot_angle):
    global yaw

    # create Twist object
    twist = Twist()
    # set up Publisher to cmd_vel topic
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    # set the update rate to 1 Hz
    rate = rospy.Rate(50)

    # get current yaw angle
    current_yaw = np.copy(yaw)
    # log the info
    #rospy.loginfo(['Current: ' + str(math.degrees(current_yaw))])
    # we are going to use complex numbers to avoid problems when the angles go from
    # 360 to 0, or from -180 to 180
    c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
    # calculate desired yaw
    target_yaw=math.radians(rot_angle)
    #target_yaw = current_yaw + math.radians(rot_angle)   -------------HHAHHAHAH
    # convert to complex notation
    c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
    #rospy.loginfo(['Desired: ' + str(math.degrees(target_yaw))])
    # divide the two complex numbers to get the change in direction
    c_change = c_target_yaw / c_yaw
    # get the sign of the imaginary component to figure out which way we have to turn
    c_change_dir = np.sign(c_change.imag)
    # set linear speed to zero so the TurtleBot rotates on the spot
    twist.linear.x = 0.0
    # set the direction to rotate
    twist.angular.z = c_change_dir * rotate_speed
    # start rotation
    time.sleep(0.2)
    pub.publish(twist)

    # we will use the c_dir_diff variable to see if we can stop rotating
    c_dir_diff = c_change_dir
    # rospy.loginfo(['c_change_dir: ' + str(c_change_dir) + ' c_dir_diff: ' + str(c_dir_diff)])
    # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
    # becomes -1.0, and vice versa
    while(c_change_dir * c_dir_diff > 0):
        # get current yaw angle
        current_yaw = np.copy(yaw)
#        if abs(current_yaw - target_yaw)<5 and twist.angular.z==c_change_dir*rotate_speed:
#            twist.angular.z=c_change_dir*rotate_speed*0.1
#            time.sleep(0.1)
#            pub.publish(twist)
        if abs(current_yaw - target_yaw)<5:
            rate=rospy.Rate(1000)
        # get the current yaw in complex form
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        #rospy.loginfo('While Yaw: %f Target Yaw: %f', math.degrees(current_yaw), math.degrees(target_yaw))
        # get difference in angle between current and target
        c_change = c_target_yaw / c_yaw
        # get the sign to see if we can stop
        c_dir_diff = np.sign(c_change.imag)
        # rospy.loginfo(['c_change_dir: ' + str(c_change_dir) + ' c_dir_diff: ' + str(c_dir_diff)])
        rate.sleep()

    #rospy.loginfo(['End Yaw: ' + str(math.degrees(current_yaw))])
    # set the rotation speed to 0
    twist.angular.z = 0.0
    # stop the rotation
    time.sleep(0.01)
    pub.publish(twist)



def rotatebotWhileMove(rot_angle):
    global yaw
    global laser_range

    # create Twist object
    twist = Twist()
    # set up Publisher to cmd_vel topic
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    # set the update rate to 1 Hz
    rate = rospy.Rate(50)

    # get current yaw angle
    current_yaw = np.copy(yaw)
    # log the info
    #rospy.loginfo(['Current: ' + str(math.degrees(current_yaw))])
    # we are going to use complex numbers to avoid problems when the angles go from
    # 360 to 0, or from -180 to 180
    c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
    # calculate desired yaw
    target_yaw = current_yaw + math.radians(rot_angle)
    # convert to complex notation
    c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
    #rospy.loginfo(['Desired: ' + str(math.degrees(target_yaw))])
    # divide the two complex numbers to get the change in direction
    c_change = c_target_yaw / c_yaw
    # get the sign of the imaginary component to figure out which way we have to turn
    c_change_dir = np.sign(c_change.imag)
    # set linear speed to zero so the TurtleBot rotates on the spot
    twist.linear.x = 0.6*linear_speed
    # set the direction to rotate
    twist.angular.z = c_change_dir * rotate_speed
    # start rotation
    time.sleep(0.2)
    pub.publish(twist)

    # we will use the c_dir_diff variable to see if we can stop rotating
    c_dir_diff = c_change_dir
    # rospy.loginfo(['c_change_dir: ' + str(c_change_dir) + ' c_dir_diff: ' + str(c_dir_diff)])
    # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
    # becomes -1.0, and vice versa
    while(c_change_dir * c_dir_diff > 0):
        # get current yaw angle
        current_yaw = np.copy(yaw)
        if abs(current_yaw - target_yaw)<5:
            rate=rospy.Rate(1000)
        # get the current yaw in complex form
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        #rospy.loginfo('While Yaw: %f Target Yaw: %f', math.degrees(current_yaw), math.degrees(target_yaw))
        # get difference in angle between current and target
        c_change = c_target_yaw / c_yaw
        # get the sign to see if we can stop
        c_dir_diff = np.sign(c_change.imag)
        # rospy.loginfo(['c_change_dir: ' + str(c_change_dir) + ' c_dir_diff: ' + str(c_dir_diff)])
        rate.sleep()

    #rospy.loginfo(['End Yaw: ' + str(math.degrees(current_yaw))])
    # set the rotation speed to 0
    
    twist.linear.x=linear_speed
    twist.angular.z = 0.0
    # stop the rotation
    time.sleep(0.01)
    pub.publish(twist)

def preciseRot():
    global laser_range
    
    twist = Twist();
    pub=rospy.Publisher("cmd_vel",Twist,queue_size=10)
    rate=rospy.Rate(50)
    
    rate.sleep()
    
    twist.angular.z=-rotate_speed
    time.sleep(0.3)
    pub.publish(twist)
    
    sideF=np.amin(laser_range[0,range(42,48,1)])
    sideB=np.amin(laser_range[0,range(122,128,1)])
    if abs(sideF-sideB)<0.02:
        twist.angular.z=0
        time.sleep(0.1)
        pub.publish(twist)
    
    
    

def findWall():
    global laser_range
    global t_yaw
    
#    rospy.init_node('findWall', anonymous=True)
#
    
#    # subscribe to odometry data
#    rospy.Subscriber('odom', Odometry, get_odom_dir)
#    # subscribe to LaserScan data
#    rospy.Subscriber('scan', LaserScan, get_laserscan)
#    # subscribe to map occupancy data
#    rospy.Subscriber('map', OccupancyGrid, get_occupancy)
#
    rate = rospy.Rate(100) # 5 Hz
#    
    rospy.loginfo("In findWall().")
    rate.sleep()
    # turn 90 degrees to the left first
    
   
    
    rotatebot(t_yaw+90)
    t_yaw+=90
    
    rospy.loginfo(str(yaw))
    
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    twist=Twist()
    a=0
    if np.amin(laser_range[0,front_angles])<=0.30:
        twist.linear.x=-linear_speed
        a=1
    else:
        twist.linear.x=linear_speed
        a=2
    twist.angular.z=0.0
    time.sleep(0.5) #1
    pub.publish(twist)
    
    while True:
        # check distances in front of bot
        fd=laser_range[0,front_angles]
        #rospy.loginfo(str(fd))
        #rospy.loginfo(str(np.amin(fd)))
        if a==1:
            if np.amin(fd)>0.20:
                break
        else:
            if np.amin(fd)<0.40:
                break
        
        
    twist.linear.x=0
    time.sleep(0.001)
    pub.publish(twist)
#    rotatebot(-90)
#    rospy.loginfo(laser_range[0,[-43,-44,-45,-46,-47]])
#    rospy.loginfo(laser_range[0,[-133,-134,-135,-136,-137]])
#    rospy.loginfo(laser_range[0,[43,44,45,46,47]])
#    rospy.loginfo(laser_range[0,[133,134,135,136,137]])
#    rospy.loginfo(laser_range[0,range(0,181,1)])
#    preciseRot()
    rotatebot(t_yaw-90)
    t_yaw-=90
    
    
def move():
    global laser_range
    global t_yaw
    global yaw
    
    rospy.init_node("move", anonymous=True)
    
    pub=rospy.Publisher("cmd_vel",Twist,queue_size=10)
    
    # subscribe to odometry data
    rospy.Subscriber('odom', Odometry, get_odom_dir)
    # subscribe to LaserScan data
    rospy.Subscriber('scan', LaserScan, get_laserscan)
    # subscribe to map occupancy data
    rospy.Subscriber('map', OccupancyGrid, get_occupancy)
    
    rate=rospy.Rate(50)
    
    rate.sleep()
    time.sleep(1)
    
    t_yaw=math.degrees(yaw)
    
    findWall()
    twist=Twist()
    
    sideD=np.amin(laser_range[0,range(87,94,1)])
    frontD=np.amin(laser_range[0,range(-3,4,1)])
    
    counter=0
    
    try:
        while True:
#            twist.linear.x=linear_speed
#            twist.angular.x=0
#            time.sleep(0.05)
#            pub.publish(twist)
        
            counter+=1
            
            rospy.loginfo(str(sideD) + " " + str(np.amin(laser_range[0,range(87,94,1)])))
            
            if np.amin(laser_range[0,range(87,94,1)])<sideD+0.05:
                if np.amin(laser_range[0,range(-3,4,1)])<0.30 and np.amin(laser_range[0,range(-3,4,1)])!=0:
                    rospy.loginfo("Dead end")
                    if np.amin(laser_range[0,range(87,94,1)])>sideD+0.05:
                        rospy.loginfo("Dead end left turn")
                        twist.linear.x=0
                        time.sleep(0.5) #0.2
                        pub.publish(twist)
                        rotatebot(t_yaw+90)
                        t_yaw+=90
                        twist.linear.x=linear_speed
                        time.sleep(0.1)
                        pub.publish(twist)
                        time.sleep((sideD/linear_speed)*2.5)
                        twist.linear.x=0
                        time.sleep(0.1)
                        pub.publish(twist)
                        time.sleep(0.1)
                        findWall()
                    else:
                        twist.linear.x=0
                        time.sleep(0.1)
                        pub.publish(twist)
                        rotatebot(t_yaw-90)
                        t_yaw-=90
                    #findWall()
                    sideD1=np.amin(laser_range[0,range(87,94,1)])
                    if sideD1!=0:
                        sideD=sideD1
                else:
                    twist.linear.x=linear_speed
                    twist.angular.x=0
                    time.sleep(0.05)
                    pub.publish(twist)
                    
            else:
                rospy.loginfo("left turn")
                twist.linear.x=0
                time.sleep(0.5) #0.2
                pub.publish(twist)
                rotatebot(t_yaw+90)
                t_yaw+=90
                twist.linear.x=linear_speed
                time.sleep(0.1)
                pub.publish(twist)
                time.sleep((sideD/linear_speed)*2.7)
                twist.linear.x=0
                time.sleep(0.1)
                pub.publish(twist)
                time.sleep(0.1)
                findWall()
                sideD1=np.amin(laser_range[0,range(87,94,1)])
                if sideD1!=0:
                    sideD=sideD1
                
            if counter==10:
                sideD1=np.amin(laser_range[0,range(87,94,1)])
                if sideD1!=0:
                    sideD=sideD1
                frontD=np.amin(laser_range[0,range(-3,4,1)])     
                counter=0
            
    except KeyboardInterrupt:
        pass
    

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
    
