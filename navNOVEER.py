#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 11 01:24:43 2020

@author: ozair
"""

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
from PIL import Image
import matplotlib.pyplot as plt
import tf2_ros

laser_range = np.array([])
occdata = []
yaw = 0.0
t_yaw=0.0
rotate_speed = 0.2
linear_speed = 0.15
stop_distance = 0.40
occ_bins = [-1, 0, 100, 101]
front_angle = 5
front_angles = range(-front_angle,front_angle+1,1)
side_angles_f = range(43,48,1)
side_angles_b = range(133,138,1)
grid_x_legit=0
grid_y_legit=0
rotated=0




def get_odom_dir(msg):
    global yaw

    orientation_quat =  msg.pose.pose.orientation
    orientation_list = [orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)


def get_laserscan(msg):
    global laser_range

    # create numpy array
    laser_range = np.array([msg.ranges])


def get_occupancy(msg, tfBuffer):
    global msg_d
    global yawtf
    global grid_x_legit
    global grid_y_legit

    global rotated

    # create numpy array
    occdata = np.array([msg.data])
    # compute histogram to identify percent of bins with -1
    occ_counts = np.histogram(occdata,occ_bins)
    # calculate total number of bins
    total_bins = msg.info.width * msg.info.height
    # log the info
#    rospy.loginfo('Width: %i Height: %i',msg.info.width,msg.info.height)
#    rospy.loginfo('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i', occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins)

    # find transform to convert map coordinates to base_link coordinates
    # lookup_transform(target_frame, source_frame, time)
    trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time(0))
    cur_pos = trans.transform.translation
    cur_rot = trans.transform.rotation
#    rospy.loginfo(['Trans: ' + str(cur_pos)])
#    rospy.loginfo(['Rot: ' + str(cur_rot)])

    # get map resolution
    map_res = msg.info.resolution
    # get map origin struct has fields of x, y, and z
    map_origin = msg.info.origin.position
    # get map grid positions for x, y position
    grid_x_legit = round((cur_pos.x - map_origin.x) / map_res)
    grid_y_legit = round(((cur_pos.y - map_origin.y) / map_res))
#    rospy.loginfo(['Grid Y: ' + str(grid_y) + ' Grid X: ' + str(grid_x)])

    # make occdata go from 0 instead of -1, reshape into 2D
    oc2 = occdata + 1
    # set all values above 1 (i.e. above 0 in the original map data, representing occupied locations)
    oc3 = (oc2>1).choose(oc2,2)
    # reshape to 2D array using column order
    odata = np.uint8(oc3.reshape(msg.info.height,msg.info.width,order='F'))
    # set current robot location to 0
    odata[grid_x_legit][grid_y_legit] = 0
    # create image from 2D array using PIL
    img = Image.fromarray(odata.astype(np.uint8))
    # find center of image
    i_centerx = msg.info.width/2
    i_centery = msg.info.height/2
    # translate by curr_pos - centerxy to make sure the rotation is performed
    # with the robot at the center
    # using tips from:
    # https://stackabuse.com/affine-image-transformations-in-python-with-numpy-pillow-and-opencv/
    translation_m = np.array([[1, 0, (i_centerx-grid_y_legit)],
                               [0, 1, (i_centery-grid_x_legit)],
                               [0, 0, 1]])
    # Image.transform function requires the matrix to be inverted
    tm_inv = np.linalg.inv(translation_m)
    # translate the image so that the robot is at the center of the image
    img_transformed = img.transform((msg.info.height, msg.info.width),
                                    Image.AFFINE,
                                    data=tm_inv.flatten()[:6],
                                    resample=Image.NEAREST)

    # convert quaternion to Euler angles
    orientation_list = [cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w]
    (roll, pitch, yawtf) = euler_from_quaternion(orientation_list)
#    rospy.loginfo(['Yaw: R: ' + str(yawtf) + ' D: ' + str(np.degrees(yawtf))])

    # rotate by 180 degrees to invert map so that the forward direction is at the top of the image
    rotated = img_transformed.rotate(np.degrees(-yawtf)+180)
    # we should now be able to access the map around the robot by converting
    # back to a numpy array: im2arr = np.array(rotated)

    # show image using grayscale map
    plt.imshow(rotated,cmap='gray')
    plt.draw_all()
    # pause to make sure the plot gets created
    plt.pause(0.00000000001)
    
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
    global yawtf
    global yaw
    global rotated
    global msg_d
    global grid_y_legit
    global grid_x_legit
    
    
    
    rospy.init_node("move", anonymous=True)
    
    pub=rospy.Publisher("cmd_vel",Twist,queue_size=10)
    
    # subscribe to odometry data
    rospy.Subscriber('odom', Odometry, get_odom_dir)
    # subscribe to LaserScan data
    rospy.Subscriber('scan', LaserScan, get_laserscan)
    # subscribe to map occupancy data
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    rospy.sleep(1.0)

    # subscribe to map occupancy data
    rospy.Subscriber('map', OccupancyGrid, get_occupancy, tfBuffer)
    
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
            r=np.array(rotated)
            
            y_count=-1
            x1=0
            x2=0
            y1=0
            y2=0
            a=False
            #the loop below is basically checking the grid 
#            for row in r:
#                y_count+=1
#                x_count=-1
#                check=False
#                for num in row:
#                    x_count+=1
##                    if num == 3:
##                        rospy.loginfo("Bot Coordinates received.")
##                        rospy.loginfo("grid_x_legit = " + str(grid_x_legit) + " grid_y_legit = " + str(grid_y_legit))
##                        grid_x=x_count
##                        grid_y=y_count
##                        check=True
##                        break
##                if check==True:
##                    break
#                rospy.loginfo(" columns = " + str(x_count))
#            rospy.loginfo(" rows = " + str(y_count))
            y_count=-1    
            grid_x=191.5
            grid_y=191.5
            if r!=[]:
                
                for row in r:
                    prev=-1
                    y_count+=1
                    x_count=-1
                    b=False
                    for num in row:
                        x_count+=1
                        if prev==0 and num==1 and 165<x_count<235 and 165<y_count<235:
                            if x1==0:
                                x1=x_count
                            x2=x_count
                            if y1==0:
                                y1=y_count
                            y2=y_count
                            a=True
                            b=True
                        prev=num
                    if a==True and b==False:
    #                    rospy.loginfo("x2-x1 = " +str(x2-x1) + " y2-y1 = " +str(y2-y1))
    #                    if abs(x2-x1)<3 and abs(y2-y1)<3:
                        if abs(x2-x1)**2 + abs(y2-y1)**2<16:
                            x1=0
                            x2=0
                            y1=0
                            y2=0
                            continue
                        break
            a1=0
            a2=0
            b1=0
            b2=0
            c=False
            y_count=-1
            if r!=[]:
                for row in r:
                    prev=-1
                    y_count+=1
                    x_count=-1
                    d=False
                    for num in row:
                        x_count+=1
                        if prev==1 and num==0 and 165<x_count<235 and 165<y_count<235:
                            if a1==0:
                                a1=x_count
                            a2=x_count
                            if b1==0:
                                b1=y_count
                            b2=y_count
                            c=True
                            d=True
                        prev=num
                    if c==True and d==False:
    #                    if abs(a2-a1)<3 and abs(b2-b1)<3:
                        if abs(a2-a1)**2 + abs(b2-b1)**2<16:
                            a1=0
                            a2=0
                            b1=0
                            b2=0
                            continue
                        break
            
            x = 0
            y = 0
            
            if (x2-x1)!=0 and (y2-y1)!=0: #and (y1<=b1 if b1>0 else True):
                x=((x2-x1)/2)+x1
                y=((y2-y1)/2)+y1
                
            elif (a2-a1)!=0 and (b2-b1)!=0: #and (y1<=b1 if b1>0 else True):
                x=((a2-a1)/2)+a1
                y=((b2-b1)/2)+b1
                
            twist.linear.x=linear_speed
            twist.angular.x=0
            time.sleep(0.05)
            pub.publish(twist)
        
            counter+=1
            
            rospy.loginfo(str(sideD) + " " + str(np.amin(laser_range[0,range(87,94,1)])))
            
            if np.amin(laser_range[0,range(87,94,1)])<sideD+0.40:
                if np.amin(laser_range[0,range(-3,4,1)])<0.40 and np.amin(laser_range[0,range(-3,4,1)])!=0:
                    rospy.loginfo("Dead end")
                    if x < grid_x:
                        rotatebot(t_yaw+90)
                        t_yaw-=90
                    
                    elif x > grid_x:
                        rotatebot(t_yaw-90)
                        t_yaw+=90
                        
            
            
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
                rotatebot(t_yaw-90)
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
    
