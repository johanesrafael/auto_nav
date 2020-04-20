#!/usr/bin/env python2
# -*- coding: utf-8 -*-
# FINAL FOR NAVIGATION AND MAPPING
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
import cv2
#from sound_play.msg import SoundRequest
#from sound_play.libsoundplay import SoundClient

laser_range = np.array([])
occdata = []
odata=[]
msg_d=[]
yaw = 0.0
yawtf=0.0
t_yaw=0.0
rotate_speed = 0.15
linear_speed = 0.11
stop_distance = 0.50
occ_bins = [-1, 0, 100, 101]
front_angle = 5
front_angles = range(-front_angle,front_angle+1,1)
side_angles_f = range(43,48,1)
side_angles_b = range(133,138,1)
grid_x_legit=0
grid_y_legit=0
height=0
width=0
rotated=0
r=[]


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
    global yawtf
    global grid_x_legit
    global grid_y_legit
    global width
    global height
    global occdata

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
#    oc2 = occdata + 1
##    odata = np.uint8(oc2.reshape(msg.info.height,msg.info.width,order='F'))
#
#    # set all values above 1 (i.e. above 0 in the original map data, representing occupied locations)
#    oc3 = (oc2>1).choose(oc2,2)
#    # reshape to 2D array using column order
#    ondata = np.uint8(oc3.reshape(msg.info.height,msg.info.width,order='F'))
#    # set current robot location to 0
#    ondata[grid_x_legit][grid_y_legit] = 0
#    # create image from 2D array using PIL
#    img = Image.fromarray(ondata.astype(np.uint8))
    # find center of image
    width = msg.info.width
    height = msg.info.height
    # translate by curr_pos - centerxy to make sure the rotation is performed
    # with the robot at the center
    # using tips from:
    # https://stackabuse.com/affine-image-transformations-in-python-with-numpy-pillow-and-opencv/
#    translation_m = np.array([[1, 0, (i_centerx-grid_y_legit)],
#                               [0, 1, (i_centery-grid_x_legit)],
#                               [0, 0, 1]])
#    # Image.transform function requires the matrix to be inverted
#    tm_inv = np.linalg.inv(translation_m)
#    # translate the image so that the robot is at the center of the image
#    img_transformed = img.transform((msg.info.height, msg.info.width),
#                                    Image.AFFINE,
#                                    data=tm_inv.flatten()[:6],
#                                    resample=Image.NEAREST)

    # convert quaternion to Euler angles
    orientation_list = [cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w]
    (roll, pitch, yawtf) = euler_from_quaternion(orientation_list)
#    rospy.loginfo(['Yaw: R: ' + str(yawtf) + ' D: ' + str(np.degrees(yawtf))])

    # rotate by 180 degrees to invert map so that the forward direction is at the top of the image
#    rotated = img_transformed.rotate(np.degrees(-yawtf)+180)
#    # we should now be able to access the map around the robot by converting
#    # back to a numpy array: im2arr = np.array(rotated)
#
#    # show image using grayscale map
#    plt.imshow(rotated,cmap='gray')
#    plt.draw_all()
#    # pause to make sure the plot gets created
#    plt.pause(0.00000000001)
    
    
def retRotated():
    global occdata
    global odata
    global msg_d
    global grid_x_legit
    global grid_y_legit
    global width
    global height
    global yawtf
    
    oc2 = occdata + 1
    odata = np.uint8(oc2.reshape(height,width,order='F'))

    # set all values above 1 (i.e. above 0 in the original map data, representing occupied locations)
    oc3 = (oc2>1).choose(oc2,2)
    # reshape to 2D array using column order
    
    ondata = np.uint8(oc3.reshape(height,width,order='F'))
    # set current robot location to 0
    ondata[grid_x_legit][grid_y_legit] = 0
    # create image from 2D array using PIL
    img = Image.fromarray(ondata.astype(np.uint8))
    # find center of image
    i_centerx = width/2
    i_centery = height/2
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
    img_transformed = img.transform((height, width),
                                    Image.AFFINE,
                                    data=tm_inv.flatten()[:6],
                                    resample=Image.NEAREST)
    rotated = img_transformed.rotate(np.degrees(-yawtf)+180)
    # we should now be able to access the map around the robot by converting
    # back to a numpy array: im2arr = np.array(rotated)

    # show image using grayscale map
    plt.imshow(rotated,cmap='gray')
    plt.draw_all()
    # pause to make sure the plot gets created
    plt.pause(0.00000000001)
    return rotated

#def get_occupancy(msg, tfBuffer):
#    global msg_d
#    global yawtf
#    global grid_x_legit
#    global grid_y_legit
#
#    global odata
#    global rotated
#
#    # create numpy array
#    occdata = np.array([msg.data])
#    # compute histogram to identify percent of bins with -1
#    occ_counts = np.histogram(occdata,occ_bins)
#    # calculate total number of bins
#    total_bins = msg.info.width * msg.info.height
#    # log the info
##    rospy.loginfo('Width: %i Height: %i',msg.info.width,msg.info.height)
##    rospy.loginfo('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i', occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins)
#
#    # find transform to convert map coordinates to base_link coordinates
#    # lookup_transform(target_frame, source_frame, time)
#    trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time(0))
#    cur_pos = trans.transform.translation
#    cur_rot = trans.transform.rotation
##    rospy.loginfo(['Trans: ' + str(cur_pos)])
##    rospy.loginfo(['Rot: ' + str(cur_rot)])
#
#    # get map resolution
#    map_res = msg.info.resolution
#    # get map origin struct has fields of x, y, and z
#    map_origin = msg.info.origin.position
#    # get map grid positions for x, y position
#    grid_x_legit = round((cur_pos.x - map_origin.x) / map_res)
#    grid_y_legit = round(((cur_pos.y - map_origin.y) / map_res))
##    rospy.loginfo(['Grid Y: ' + str(grid_y) + ' Grid X: ' + str(grid_x)])
#
#    # make occdata go from 0 instead of -1, reshape into 2D
#    oc2 = occdata + 1
#    odata = np.uint8(oc2.reshape(msg.info.height,msg.info.width,order='F'))
#
#    # set all values above 1 (i.e. above 0 in the original map data, representing occupied locations)
#    oc3 = (oc2>1).choose(oc2,2)
#    # reshape to 2D array using column order
#    ondata = np.uint8(oc3.reshape(msg.info.height,msg.info.width,order='F'))
#    # set current robot location to 0
#    ondata[grid_x_legit][grid_y_legit] = 0
#    # create image from 2D array using PIL
#    img = Image.fromarray(ondata.astype(np.uint8))
#    # find center of image
#    i_centerx = msg.info.width/2
#    i_centery = msg.info.height/2
#    # translate by curr_pos - centerxy to make sure the rotation is performed
#    # with the robot at the center
#    # using tips from:
#    # https://stackabuse.com/affine-image-transformations-in-python-with-numpy-pillow-and-opencv/
#    translation_m = np.array([[1, 0, (i_centerx-grid_y_legit)],
#                               [0, 1, (i_centery-grid_x_legit)],
#                               [0, 0, 1]])
#    # Image.transform function requires the matrix to be inverted
#    tm_inv = np.linalg.inv(translation_m)
#    # translate the image so that the robot is at the center of the image
#    img_transformed = img.transform((msg.info.height, msg.info.width),
#                                    Image.AFFINE,
#                                    data=tm_inv.flatten()[:6],
#                                    resample=Image.NEAREST)
#
#    # convert quaternion to Euler angles
#    orientation_list = [cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w]
#    (roll, pitch, yawtf) = euler_from_quaternion(orientation_list)
##    rospy.loginfo(['Yaw: R: ' + str(yawtf) + ' D: ' + str(np.degrees(yawtf))])
#
#    # rotate by 180 degrees to invert map so that the forward direction is at the top of the image
#    rotated = img_transformed.rotate(np.degrees(-yawtf)+180)
#    # we should now be able to access the map around the robot by converting
#    # back to a numpy array: im2arr = np.array(rotated)
#
#    # show image using grayscale map
#    plt.imshow(rotated,cmap='gray')
#    plt.draw_all()
#    # pause to make sure the plot gets created
#    plt.pause(0.00000000001)


def rotatebotSlow(rot_angle):
    global yaw

    # create Twist object
    twist = Twist()
    # set up Publisher to cmd_vel topic
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    # set the update rate to 1 Hz
    rate = rospy.Rate(1000)

    # get current yaw angle
    current_yaw = np.copy(yaw)
    # log the info
#    rospy.loginfo(['Current: ' + str(math.degrees(current_yaw))])
    # we are going to use complex numbers to avoid problems when the angles go from
    # 360 to 0, or from -180 to 180
    c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
    # calculate desired yaw
    target_yaw = current_yaw + math.radians(rot_angle)
    # convert to complex notation
    c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
#    rospy.loginfo(['Desired: ' + str(math.degrees(cmath.phase(c_target_yaw)))])
    # divide the two complex numbers to get the change in direction
    c_change = c_target_yaw / c_yaw
    # get the sign of the imaginary component to figure out which way we have to turn
    c_change_dir = np.sign(c_change.imag)
    # set linear speed to zero so the TurtleBot rotates on the spot
    twist.linear.x = 0.0
    # set the direction to rotate
    twist.angular.z = c_change_dir * 0.08
    # start rotation
    pub.publish(twist)

    # we will use the c_dir_diff variable to see if we can stop rotating
    c_dir_diff = c_change_dir
    # rospy.loginfo(['c_change_dir: ' + str(c_change_dir) + ' c_dir_diff: ' + str(c_dir_diff)])
    # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
    # becomes -1.0, and vice versa
    while(c_change_dir * c_dir_diff > 0):
        # get current yaw angle
        current_yaw = np.copy(yaw)
        # get the current yaw in complex form
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
#        rospy.loginfo('While Yaw: %f Target Yaw: %f', math.degrees(current_yaw), math.degrees(target_yaw))
        # get difference in angle between current and target
        c_change = c_target_yaw / c_yaw
        # get the sign to see if we can stop
        c_dir_diff = np.sign(c_change.imag)
        # rospy.loginfo(['c_change_dir: ' + str(c_change_dir) + ' c_dir_diff: ' + str(c_dir_diff)])
        rate.sleep()

#    rospy.loginfo(['End Yaw: ' + str(math.degrees(current_yaw))])
    # set the rotation speed to 0
    twist.angular.z = 0.0
    # stop the rotation
    time.sleep(1)
    pub.publish(twist)

def rotatebot(rot_angle):
    global yaw

    # create Twist object
    twist = Twist()
    # set up Publisher to cmd_vel topic
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    # set the update rate to 1 Hz
    rate = rospy.Rate(1000)

    # get current yaw angle
    current_yaw = np.copy(yaw)
    # log the info
#    rospy.loginfo(['Current: ' + str(math.degrees(current_yaw))])
    # we are going to use complex numbers to avoid problems when the angles go from
    # 360 to 0, or from -180 to 180
    c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
    # calculate desired yaw
    target_yaw = current_yaw + math.radians(rot_angle)
    # convert to complex notation
    c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
#    rospy.loginfo(['Desired: ' + str(math.degrees(cmath.phase(c_target_yaw)))])
    # divide the two complex numbers to get the change in direction
    c_change = c_target_yaw / c_yaw
    # get the sign of the imaginary component to figure out which way we have to turn
    c_change_dir = np.sign(c_change.imag)
    # set linear speed to zero so the TurtleBot rotates on the spot
    twist.linear.x = 0.0
    # set the direction to rotate
    twist.angular.z = c_change_dir * rotate_speed
    # start rotation
    pub.publish(twist)

    # we will use the c_dir_diff variable to see if we can stop rotating
    c_dir_diff = c_change_dir
    # rospy.loginfo(['c_change_dir: ' + str(c_change_dir) + ' c_dir_diff: ' + str(c_dir_diff)])
    # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
    # becomes -1.0, and vice versa
    while(c_change_dir * c_dir_diff > 0):
        # get current yaw angle
        current_yaw = np.copy(yaw)
        # get the current yaw in complex form
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
#        rospy.loginfo('While Yaw: %f Target Yaw: %f', math.degrees(current_yaw), math.degrees(target_yaw))
        # get difference in angle between current and target
        c_change = c_target_yaw / c_yaw
        # get the sign to see if we can stop
        c_dir_diff = np.sign(c_change.imag)
        # rospy.loginfo(['c_change_dir: ' + str(c_change_dir) + ' c_dir_diff: ' + str(c_dir_diff)])
        rate.sleep()

#    rospy.loginfo(['End Yaw: ' + str(math.degrees(current_yaw))])
    # set the rotation speed to 0
    twist.angular.z = 0.0
    # stop the rotation
    time.sleep(1)
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
    rate = rospy.Rate(200) # 5 Hz
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
    if np.amin(laser_range[0,front_angles])<=0.25:
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
            if np.amin(fd)<0.25:
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
    
    
def moveTo(D):
    global laser_range
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(200)
    
    rospy.loginfo("In moveTo")
    
    rate.sleep()
    twist=Twist()
    
#    rotatebot(-5)
#    if laser_range[0,0]>D+0.1:
#        D=laser_range[0,0]
#    else:
#        rotatebot(10)
#        if laser_range[0,0]>D+0.1:
#            D=laser_range[0,0]
#        else:
#            rotatebot(-5) 
    
    twist.linear.x=linear_speed
    twist.angular.z=0
    time.sleep(0.1)
    pub.publish(twist)
    rospy.loginfo(" D = " +str(D))
    rospy.loginfo(" time.sleep = "  + str((D/linear_speed) *0.6))
    time.sleep((D/linear_speed) *0.6)
    #time.sleep(1)
    twist.linear.x=0
    time.sleep(0.1)
    pub.publish(twist)
    
    if np.amin(laser_range[0,range(-3,4)])<0.2:
        twist.linear.x=-linear_speed
        time.sleep(0.1)
        pub.publish(twist)
        time.sleep(1)
        twist.linear.x=0
        time.sleep(0.1)
        pub.publish(twist)
    

def getUA():
    global rotated
    r=np.array(rotated)
    y_count=-1    
    x1=0
    x2=0
    y1=0
    y2=0
    grid_x=191.5
    grid_y=191.5
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
#                   if abs(a2-a1)<3 and abs(b2-b1)<3:
            if abs(a2-a1)**2 + abs(b2-b1)**2<16:
                a1=0
                a2=0
                b1=0
                b2=0
                continue
            break
    
        
def closure(mapdata):
    
    # This function checks if mapdata contains a closed contour. The function
    # assumes that the raw map data from SLAM has been modified so that
    # -1 (unmapped) is now 0, and 0 (unoccupied) is now 1, and the occupied
    # values go from 1 to 101.

    # According to: https://stackoverflow.com/questions/17479606/detect-closed-contours?rq=1
    # closed contours have larger areas than arc length, while open contours have larger
    # arc length than area. But in my experience, open contours can have areas larger than
    # the arc length, but closed contours tend to have areas much larger than the arc length
    # So, we will check for contour closure by checking if any of the contours
    # have areas that are more than 10 times larger than the arc length
    # This value may need to be adjusted with more testing.
    ALTHRESH = 10
    # We will slightly fill in the contours to make them easier to detect
    DILATE_PIXELS = 3

    # assumes mapdata is uint8 and consists of 0 (unmapped), 1 (unoccupied),
    # and other positive values up to 101 (occupied)
    # so we will apply a threshold of 2 to create a binary image with the
    # occupied pixels set to 255 and everything else is set to 0
    # we will use OpenCV's threshold function for this
    ret,img2 = cv2.threshold(mapdata,2,255,0)
    # we will perform some erosion and dilation to fill out the contours a
    # little bit
    element = cv2.getStructuringElement(cv2.MORPH_CROSS,(DILATE_PIXELS,DILATE_PIXELS))
    # img3 = cv2.erode(img2,element)
    img4 = cv2.dilate(img2,element)
    # use OpenCV's findContours function to identify contours
    # OpenCV version 3 changed the number of return arguments, so we
    # need to check the version of OpenCV installed so we know which argument
    # to grab
    fc = cv2.findContours(img4, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    (major, minor, _) = cv2.__version__.split(".")
    if(major == '3'):
        contours = fc[1]
    else:
        contours = fc[0]
    # find number of contours returned
    lc = len(contours)
    # rospy.loginfo('# Contours: %s', str(lc))
    # create array to compute ratio of area to arc length
    cAL = np.zeros((lc,2))
    for i in range(lc):
        cAL[i,0] = cv2.contourArea(contours[i])
        cAL[i,1] = cv2.arcLength(contours[i], True)

    # closed contours tend to have a much higher area to arc length ratio,
    # so if there are no contours with high ratios, we can safely say
    # there are no closed contours
    cALratio = cAL[:,0]/cAL[:,1]
    rospy.loginfo('Closure: %s', str(cALratio))
    
    if cALratio[0] > ALTHRESH:
        return True
    else:
        return False

    
def move():
    global laser_range
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
    
    rate=rospy.Rate(200)
    
    rate.sleep()
    
    t_yaw=math.degrees(yaw)
    
    time.sleep(1)
    
    twist=Twist()
#    twist.linear.x=0.1
#    twist.angular.z=0
#    time.sleep(0.1)
#    pub.publish(twist)
    
#    rospy.loginfo(np.argmin(laser_range))
#    rotatebot(-170)
#    while True:
#        rospy.loginfo(" grid_x_legit = " + str(grid_x_legit) + " grid_y_legit = " + str(grid_y_legit))
    
    contourCheck=1
    start_time = time.time()
    
    try:
        
        while contourCheck==1:
            
            r=np.array(retRotated())
#            closure(odata)
            
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
            for row in r:
                prev=-1
                y_count+=1
                x_count=-1
                b=False
                for i,num in enumerate(row):
                    x_count+=1
                    if prev==0 and num==1 and 165<x_count<235 and 165<y_count<235 and row[i+1]==1 and row[i+2]==1 and row[i+3]==1 and row[i+4]==1:
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
            for row in r:
                prev=-1
                y_count+=1
                x_count=-1
                d=False
                for i,num in enumerate(row):
                    x_count+=1
                    if prev==1 and num==0 and 165<x_count<235 and 165<y_count<235 and row[i-1]==1 and row[i-2]==1 and row[i-3]==1 and row[i-4]==1:
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
#            x_count=-1
#            i1=0
#            i2=0
#            j1=0
#            j2=0
#            e=False
#            for column in r.T:
#                x_count+=1
#                prev=-1
#                y_count=-1
#                f=False
#                for num in column:
#                    y_count+=1
#                    if prev==1 and num==0:
#                        if i1==0:
#                            i1=x_count
#                        i2=x_count
#                        if j1==0:
#                            j1=y_count
#                        j2=y_count
#                        e=True
#                        f=True
#                    prev=num
#                if e==True and f==False:
#                    break
                        
            rospy.loginfo("Reached.")
            rospy.loginfo("x1 = " + str(x1) + "  x2 = " + str(x2) + "  y1 = " +str(y1) + "  y2 = " + str(y2))
            rospy.loginfo("a1 = " + str(a1) + "  a2 = " + str(a2) + "  b1 = " +str(b1) + "  b2 = " + str(b2))
#            rospy.loginfo("i1 = " + str(i1) + " i2 = " + str(i2) + " j1 = " + str(j1) + " j2 = " + str(j2))
            if (x2-x1)!=0 and (y2-y1)!=0: #and (y1<=b1 if b1>0 else True):
                x=((x2-x1)/2)+x1
                y=((y2-y1)/2)+y1
#                y2=383-y2
#                y1=383-y1
#                x=0
#                y=0
#                if (x1-grid_x)**2 + (y1-grid_y)**2 < (x2-grid_x)**2 + (y2-grid_y)**2:
#                    x=x1
#                    y=y1
#                else:
#                    x=x2
#                    y=y2
                special=False
                angle=0
                if x<grid_x and y<grid_y:
                    #angle=90-math.degrees(math.atan(float(abs(y-grid_y))/abs(x-grid_x)))
                    angle=90-math.degrees(math.atan(float(abs(y2-y1))/abs(x2-x1)))
                    if abs(y-grid_y)<5:
                        special=True
                elif x<grid_x and y>grid_y:
                    #angle=90+math.degrees(math.atan(float(abs(y-grid_y))/abs(x-grid_x)))
                    angle=90+math.degrees(math.atan(float(abs(y2-y1))/abs(x2-x1)))
                elif x>grid_x and y<grid_y:
                    #angle = -(90-math.degrees(math.atan(float(abs(y-grid_y))/abs(x-grid_x))))
                    angle=-(90-math.degrees(math.atan(float(abs(y2-y1))/abs(x2-x1))))
                elif x>grid_x and y>grid_y:
                    #angle = -(90+math.degrees(math.atan(float(abs(y-grid_y))/abs(x-grid_x))))
                    angle=-(90+math.degrees(math.atan(float(abs(y2-y1))/abs(x2-x1))))
                    
                    special=True

                rospy.loginfo("x = " + str(x) + "  x_grid = " + str(grid_x) + "  y = " +str(y) + "  y_grid = " + str(grid_y))
                try:
                    rospy.loginfo("angle = " + str(angle))
                    #rospy.loginfo(laser_range[0,range(int(angle)-25,int(angle)+21)])
#                    lr2i = np.argmax(laser_range[0,range(int(angle)-20,int(angle)-10)+range(int(angle)+10,int(angle)+21)])
#                    D=laser_range[0,lr2i]
#                    rospy.loginfo("D = " + str(D))
#                    rotatebot(float(lr2i))
#                    rospy.loginfo(range(int(angle)-25,int(angle)-20)+range(int(angle)+20,int(angle)+26))
                    arr=[]
                    if special==True:
                        if angle>0:
                            arr=range(int(angle)-25,int(angle)-20)
                        else:
                            arr=range(int(angle)+20,int(angle)+26)
                    else:
#                        arr=range(int(angle)-25,int(angle)-20)+range(int(angle)+20,int(angle)+26)

                        if angle<0:
                            if angle-25<-180:
                                arr=range(int(angle)+20,int(angle)+26) 
                            else:
                                arr=range(int(angle)-25,int(angle)-20)+range(int(angle)+20,int(angle)+26)
                        elif angle>0:
                            if angle+25>180:
                                arr=range(int(angle)-25,int(angle)-20) 
                            else:
                                arr=range(int(angle)-25,int(angle)-20)+range(int(angle)+20,int(angle)+26)
                    
                    rospy.loginfo(arr)
                    
                    distarray=[]
                    max1=0
                    arg=400
                    zerocheck=False
                    for w in arr:
                        dist=laser_range[0,w]
                        if dist==0 or dist==float("inf"):
                            rospy.loginfo("Found infinite distance at arg = " + str(w))
                            arg=w
                            zerocheck=True
                            break
                        distarray.append(dist)
                        if dist>max1:
                            max1=dist
                            arg=w
                    rospy.loginfo("arg = " + str(arg))
                    
                    rotatebot(float(arg))
                    time.sleep(0.1)
                    
                    if arg>0:
                        if arg>90:
                            rotatebotSlow(-3)
                        else:
                            rotatebotSlow(-(arg//50)-1)
                    else:
                        if arg<-90:
                            rotatebotSlow(3)
                        else:
                            
                            rotatebotSlow(-(arg//50)+1)
                     
                    D=laser_range[0,0]
#                    rotatebot(-5)
#                    if laser_range[0,0]>D+0.1 or (laser_range[0,0]==0) or (laser_range[0,0]==float("inf")):
#                        D=laser_range[0,0]
#                    else:
#                        rotatebot(10)
#                        if laser_range[0,0]>D+0.1 or (laser_range[0,0]==0) or (laser_range[0,0]==float("inf")):
#                            D=laser_range[0,0]
#                        else:
#                            rotatebot(-5) 

                    rospy.loginfo("D = " + str(D))
                    if D==0 or D==float("inf"):
                        rospy.loginfo("In D==inf")

                       # moveTo(2)
                        twist.linear.x=linear_speed
                        time.sleep(0.1)
                        pub.publish(twist)
                        while D==0 or D==float("inf"):
                            D=laser_range[0,0]
                            
                        twist.linear.x=0
                        time.sleep(0.1)
                        pub.publish(twist)
                        moveTo(D)
                       
                    else:
                        moveTo(D)
                except ValueError:
                    # in case laser_range is empty
                    rospy.loginfo("ValueError for lr2i")
                    lr2i = 0
                    
            elif (a2-a1)!=0 and (b2-b1)!=0:
                x=((a2-a1)/2)+a1
                y=((b2-b1)/2)+b1
#                x=0
#                y=0
#                b2=383-b2
#                b1=383-b1
#                if (a1-grid_x)**2 + (b1-grid_y)**2 < (a2-grid_x)**2 + (b2-grid_y)**2:
#                    x=a1
#                    y=b1
#                else:
#                    x=a2
#                    y=b2
                special=False
                angle=0
                if x<grid_x and y<grid_y:
                    #angle=90-math.degrees(math.atan(float(abs(y-grid_y))/abs(x-grid_x)))
                    angle=90-math.degrees(math.atan(float(abs(b2-b1))/abs(a2-a1)))
                elif x<grid_x and y>grid_y:
                    #angle=90+math.degrees(math.atan(float(abs(y-grid_y))/abs(x-grid_x)))
                    angle=90+math.degrees(math.atan(float(abs(b2-b1))/abs(a2-a1)))
                    special=True
                elif x>grid_x and y<grid_y:
                    #angle = -(90-math.degrees(math.atan(float(abs(y-grid_y))/abs(x-grid_x))))
                    angle=-(90-math.degrees(math.atan(float(abs(b2-b1))/abs(a2-a1))))
                    if abs(y-grid_x)<5:
                        special=True
                elif x>grid_x and y>grid_y:
                    #angle = -(90+math.degrees(math.atan(float(abs(y-grid_y))/abs(x-grid_x))))
                    angle=-(90+math.degrees(math.atan(float(abs(b2-b1))/abs(a2-a1))))

                rospy.loginfo("x = " + str(x) + "  x_grid = " + str(grid_x) + "  y = " +str(y) + "  y_grid = " + str(grid_y))
                rospy.loginfo("actual angle = " + str(math.degrees(math.atan(float(abs(b2-b1))/abs(a2-a1)))))
                try:
                    rospy.loginfo("angle = " + str(angle))
                    #rospy.loginfo(laser_range[0,range(int(angle)-20,int(angle)+21)])
#                    rospy.loginfo(range(int(angle)-25,int(angle)-20)+range(int(angle)+20,int(angle)+26))
#                    lr2i = np.argmax(laser_range[0,range(int(angle)-20,int(angle)-10)+range(int(angle)+10,int(angle)+21)])
#                    rospy.loginfo(str(lr2i))
                    arr=[]
                    if special==True:
                        if angle>0:
                            arr=range(int(angle)-25,int(angle)-20)
                        else:
                            arr=range(int(angle)+20,int(angle)+26)
                    else:
                        arr=range(int(angle)-25,int(angle)-20)+range(int(angle)+20,int(angle)+26)
                        if angle<0:
                            if angle-25<-180:
                                arr=range(int(angle)+20,int(angle)+26) 
                            else:
                                arr=range(int(angle)-25,int(angle)-20)+range(int(angle)+20,int(angle)+26)
                        elif angle>0:
                            if angle+25>180:
                                arr=range(int(angle)-25,int(angle)-20) 
                            else:
                                arr=range(int(angle)-25,int(angle)-20)+range(int(angle)+20,int(angle)+26)
                    rospy.loginfo(arr)
                    distarray=[]
                    max1=0
                    arg=400
                    zerocheck=False
                    for w in arr:
                        dist=laser_range[0,w]
                        if dist==0 or dist==float("inf"):
                            rospy.loginfo("Found infinite distance at arg = " + str(w))
                            arg=w
                            zerocheck=True
                            break
                        distarray.append(dist)
                        if dist>max1:
                            max1=dist
                            arg=w
                    rospy.loginfo("arg = " + str(arg))
                 
                    rotatebot(float(arg))
                    time.sleep(0.1)
                    
                    
#                    if np.amin(laser_range[0,range(85,95)])<0.3:
#                        if np.amin(laser_range[0,range(85,95)])<np.amin(laser_range[0,range(265,275)]):
#                            rotatebot(-4)
#                        else:
#                            rotatebot(4)
#                    elif np.amin(laser_range[0,range(265,275)])<0.3:
#                        if np.amin(laser_range[0,range(85,95)])>np.amin(laser_range[0,range(265,275)]):
#                            rotatebot(4)
#                        else:
#                            rotatebot(-4)
                    if arg>0:
                        if arg>90:
                            rotatebotSlow(-3)
                        else:
                            rotatebotSlow(-(arg//50)-1)
                    else:
                        if arg<-90:
                            rotatebotSlow(3)
                        else:
                            
                            rotatebotSlow(-(arg//50)+1)
                   


                    D=laser_range[0,0]
#                    rotatebot(-5)
#                    if laser_range[0,0]>D+0.1 or (laser_range[0,0]==0) or (laser_range[0,0]==float("inf")):
#                        D=laser_range[0,0]
#                    else:
#                        rotatebot(10)
#                        if laser_range[0,0]>D+0.1 or (laser_range[0,0]==0) or (laser_range[0,0]==float("inf")):
#                            D=laser_range[0,0]
#                        else:
#                            rotatebot(-5) 
                    
                    rospy.loginfo("D = " + str(D))
                    if D==0 or D==float("inf"):
                        rospy.loginfo("In D==inf")
                        twist.linear.x=linear_speed
                        time.sleep(0.1)
                        pub.publish(twist)
                        while D==0 or D==float("inf"):
                            D=laser_range[0,0]
                            
                        twist.linear.x=0
                        time.sleep(0.1)
                        pub.publish(twist)
                        moveTo(D)
                    else:
                        moveTo(D)
                except ValueError:
                    # in case laser_range is empty
                    rospy.loginfo("ValueError for lr2i")
                    lr2i = 0
#                angle=90-math.degrees(math.atan(float(abs(y-grid_y))/abs(x-grid_x)))
#                rospy.loginfo(str(angle))
#                D=laser_range[0,int(angle)]
#                rospy.loginfo("D is " +str(D))
#                if angle>30:
#                    rotatebot(angle-25)
#                else:
#                    rotatebot(angle-0.8*angle)
#                moveTo(D)
                    
#            elif (i2-i1)!=0:
#                x = ((i2-i1)/2)+i1
#                rospy.loginfo("x = " + str(x) + "  x_grid = " + str(grid_x))
#                angle=0
#                if x>grid_x:
#                    angle=-70
#                if x<grid_x:
#                    angle=70
#                rospy.loginfo("angle = " +str(angle))
#                D=laser_range[0,angle]
#                rospy.loginfo("D = " + str(D))
#                rotatebot(float(angle))
#                moveTo(D)
                
            else:
                rospy.loginfo("In last case")
                x1=0
                x2=0
                y1=0
                y2=0
                y_count=-1
                a=False
                for row in r:
                    prev=-1
                    y_count+=1
                    x_count=-1
                    b=False
                    for i,num in enumerate(row):
                        x_count+=1
                        if prev==0 and num==1 and row[i+1]==1 and row[i+2]==1 and row[i+3]==1 and row[i+4]==1:
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
                        if abs(x2-x1)**2+abs(y2-y1)**2<16:
                       
                            x1=0
                            x2=0
                            y1=0
                            y2=0
                            continue
                        break
                rospy.loginfo("x1 = " + str(x1) + "  x2 = " + str(x2) + "  y1 = " +str(y1) + "  y2 = " + str(y2))
                a1=0
                a2=0
                a1=0
                a2=0
                y_count=-1
                a=False
                for row in r:
                    prev=-1
                    y_count+=1
                    x_count=-1
                    b=False
                    for i,num in enumerate(row):
                        x_count+=1
                        if prev==0 and num==1 and row[i-1]==1 and row[i-2]==1 and row[i-3]==1 and row[i-4]==1:
                            if a1==0:
                                a1=x_count
                            a2=x_count
                            if b1==0:
                                b1=y_count
                            b2=y_count
                            a=True
                            b=True
                        prev=num
                    if a==True and b==False:
#                    rospy.loginfo("x2-x1 = " +str(x2-x1) + " y2-y1 = " +str(y2-y1))
                        if abs(a2-a1)**2+abs(b2-b1)**2<16:
                       
                            a1=0
                            a2=0
                            b1=0
                            b2=0
                            continue
                        break
                rospy.loginfo("a1 = " + str(a1) + "  a2 = " + str(a2) + "  b1 = " +str(b1) + "  b2 = " + str(b2))

                if (x2-x1)!=0 and (y2-y1)!=0:
                
                    x=((x2-x1)/2)+x1
                    y=((y2-y1)/2)+y1
                    special=False
#                    if (x-grid_x)!=0 and (y-grid_y)!=0:
                    if x<grid_x and y<grid_y:
                        #angle=90-math.degrees(math.atan(float(abs(y-grid_y))/abs(x-grid_x)))
                        angle=90-math.degrees(math.atan(float(abs(y-grid_y))/abs(x-grid_x)))
                        
                    elif x<grid_x and y>grid_y:
                            #angle=90+math.degrees(math.atan(float(abs(y-grid_y))/abs(x-grid_x)))
                        angle=90+math.degrees(math.atan(float(abs(y-grid_y))/abs(x-grid_x)))
                    elif x>grid_x and y<grid_y:
                                #angle = -(90-math.degrees(math.atan(float(abs(y-grid_y))/abs(x-grid_x))))
                        angle=-(90-math.degrees(math.atan(float(abs(y-grid_y))/abs(x-grid_x))))
                    elif x>grid_x and y>grid_y:
                    #angle = -(90+math.degrees(math.atan(float(abs(y-grid_y))/abs(x-grid_x))))
                        angle=-(90+math.degrees(math.atan(float(abs(y-grid_y))/abs(x-grid_x))))
                    
                        special=True
                    rospy.loginfo("x = " + str(x) + "  x_grid = " + str(grid_x) + "  y = " +str(y) + "  y_grid = " + str(grid_y))
#                    rospy.loginfo("actual angle = " + str(math.degrees(math.atan(float(abs(b2-b1))/abs(a2-a1)))))
                    try:
                        rospy.loginfo("angle = " + str(angle))
                    #rospy.loginfo(laser_range[0,range(int(angle)-20,int(angle)+21)])
#                    rospy.loginfo(range(int(angle)-25,int(angle)-20)+range(int(angle)+20,int(angle)+26))
#                    lr2i = np.argmax(laser_range[0,range(int(angle)-20,int(angle)-10)+range(int(angle)+10,int(angle)+21)])
#                    rospy.loginfo(str(lr2i))
                        arr=[]
                        if special==True:
                            if angle>0:
                                arr=range(int(angle)-25,int(angle)-20)
                            else:
                                arr=range(int(angle)+20,int(angle)+26)
                        else:
                            arr=range(int(angle)-25,int(angle)-20)+range(int(angle)+20,int(angle)+26)
                            if angle<0:
                                if angle-25<-180:
                                    arr=range(int(angle)+20,int(angle)+26)
                                else:
                                    arr=range(int(angle)-25,int(angle)-20)+range(int(angle)+20,int(angle)+26)
                            elif angle>0:
                                if angle+25>180:
                                    arr=range(int(angle)-25,int(angle)-20) 
                                else:
                                    arr=range(int(angle)-25,int(angle)-20)+range(int(angle)+20,int(angle)+26)
                        rospy.loginfo(arr)
                        distarray=[]
                        max1=0
                        arg=400
                        zerocheck=False
                        for w in arr:
                            dist=laser_range[0,w]
                            if dist==0 or dist==float("inf"):
                                rospy.loginfo("Found infinite distance at arg = " + str(w))
                                arg=w
                                zerocheck=True
                                break
                            distarray.append(dist)
                            if dist>max1:
                                max1=dist
                                arg=w
                        rospy.loginfo("arg = " + str(arg))
                 
                        rotatebot(float(arg))
                        time.sleep(0.1)
                        if arg>0:
                            if arg>90:
                                rotatebotSlow(-3)
                            else:
                                rotatebotSlow(-(arg//50)-1)
                        else:
                            if arg<-90:
                                rotatebotSlow(3)
                            else:
                            
                                rotatebotSlow(-(arg//50)+1)

       
    
                        D=laser_range[0,0]
#                        rotatebot(-5)
#                        if laser_range[0,0]>D+0.1 or (laser_range[0,0]==0) or (laser_range[0,0]==float("inf")):
#                            D=laser_range[0,0]
#                        else:
#                            rotatebot(10)
#                            if laser_range[0,0]>D+0.1 or (laser_range[0,0]==0) or (laser_range[0,0]==float("inf")):
#                                D=laser_range[0,0]
#                            else:
#                                rotatebot(-5) 
                        
                        rospy.loginfo("D = " + str(D))
                        if D==0 or D==float("inf"):
                            rospy.loginfo("In D==inf")
                            twist.linear.x=linear_speed
                            time.sleep(0.1)
                            pub.publish(twist)
                            while D==0 or D==float("inf"):
                                D=laser_range[0,0]
                            
                            twist.linear.x=0
                            time.sleep(0.1)
                            pub.publish(twist)
                            moveTo(D)
                        else:
                            moveTo(D)
                    except ValueError:
                    # in case laser_range is empty
                        rospy.loginfo("ValueError for lr2i")
                        lr2i = 0
                        
                elif (a2-a1)!=0 and (b2-b1)!=0:
                
                    x=((a2-a1)/2)+a1
                    y=((b2-b1)/2)+b1
                    special=False
#                    if (x-grid_x)!=0 and (y-grid_y)!=0:
                    if x<grid_x and y<grid_y:
                        #angle=90-math.degrees(math.atan(float(abs(y-grid_y))/abs(x-grid_x)))
                        angle=90-math.degrees(math.atan(float(abs(y-grid_y))/abs(x-grid_x)))
                        
                    elif x<grid_x and y>grid_y:
                            #angle=90+math.degrees(math.atan(float(abs(y-grid_y))/abs(x-grid_x)))
                        angle=90+math.degrees(math.atan(float(abs(y-grid_y))/abs(x-grid_x)))
                    elif x>grid_x and y<grid_y:
                                #angle = -(90-math.degrees(math.atan(float(abs(y-grid_y))/abs(x-grid_x))))
                        angle=-(90-math.degrees(math.atan(float(abs(y-grid_y))/abs(x-grid_x))))
                    elif x>grid_x and y>grid_y:
                    #angle = -(90+math.degrees(math.atan(float(abs(y-grid_y))/abs(x-grid_x))))
                        angle=-(90+math.degrees(math.atan(float(abs(y-grid_y))/abs(x-grid_x))))
                    
                        special=True
                    rospy.loginfo("x = " + str(x) + "  x_grid = " + str(grid_x) + "  y = " +str(y) + "  y_grid = " + str(grid_y))
#                    rospy.loginfo("actual angle = " + str(math.degrees(math.atan(float(abs(b2-b1))/abs(a2-a1)))))
                    try:
                        rospy.loginfo("angle = " + str(angle))
                    #rospy.loginfo(laser_range[0,range(int(angle)-20,int(angle)+21)])
#                    rospy.loginfo(range(int(angle)-25,int(angle)-20)+range(int(angle)+20,int(angle)+26))
#                    lr2i = np.argmax(laser_range[0,range(int(angle)-20,int(angle)-10)+range(int(angle)+10,int(angle)+21)])
#                    rospy.loginfo(str(lr2i))
                        arr=[]
                        if special==True:
                            if angle>0:
                                arr=range(int(angle)-25,int(angle)-20)
                            else:
                                arr=range(int(angle)+20,int(angle)+26)
                        else:
                            arr=range(int(angle)-25,int(angle)-20)+range(int(angle)+20,int(angle)+26)
                            if angle<0:
                                if angle-25<-180:
                                    arr=range(int(angle)+20,int(angle)+26) 
                                else:
                                    arr=range(int(angle)-25,int(angle)-20)+range(int(angle)+20,int(angle)+26)
                            elif angle>0:
                                if angle+25>180:
                                    arr=range(int(angle)-25,int(angle)-20) 
                                else:
                                    arr=range(int(angle)-25,int(angle)-20)+range(int(angle)+20,int(angle)+26)
                        rospy.loginfo(arr)
                        distarray=[]
                        max1=0
                        arg=400
                        zerocheck=False
                        for w in arr:
                            dist=laser_range[0,w]
                            if dist==0 or dist==float("inf"):
                                rospy.loginfo("Found infinite distance at arg = " + str(w))
                                arg=w
                                zerocheck=True
                                break
                            distarray.append(dist)
                            if dist>max1:
                                max1=dist
                                arg=w
                        rospy.loginfo("arg = " + str(arg))
                 
                        rotatebot(float(arg))
                        time.sleep(0.1)
                        if arg>0:
                            if arg>90:
                                rotatebotSlow(-3)
                            else:
                                rotatebotSlow(-(arg//50)-1)
                        else:
                            if arg<-90:
                                rotatebotSlow(3)
                            else:
                            
                                rotatebotSlow(-(arg//50)+1)
                        


                        D=laser_range[0,0]
#                        rotatebot(-5)
#                        if laser_range[0,0]>D+0.1 or (laser_range[0,0]==0) or (laser_range[0,0]==float("inf")):
#                            D=laser_range[0,0]
#                        else:
#                            rotatebot(10)
#                            if laser_range[0,0]>D+0.1 or (laser_range[0,0]==0) or (laser_range[0,0]==float("inf")):
#                                D=laser_range[0,0]
#                            else:
#                                rotatebot(-5) 
                        
                        rospy.loginfo("D = " + str(D))
                        if D==0 or D==float("inf"):
                            rospy.loginfo("In D==inf")
                            twist.linear.x=linear_speed
                            time.sleep(0.1)
                            pub.publish(twist)
                            while D==0 or D==float("inf"):
                                D=laser_range[0,0]
                            
                            twist.linear.x=0
                            time.sleep(0.1)
                            pub.publish(twist)
                            moveTo(D)
                        else:
                            moveTo(D)
                    except ValueError:
                    # in case laser_range is empty
                        rospy.loginfo("ValueError for lr2i")
                        lr2i = 0
                else:
                    if closure(odata):
                        # map is complete, so save current time into file
                        rospy.loginfo("Closure activated")
                        with open("maptime.txt", "w") as f:
                            f.write("Elapsed Time: " + str(time.time() - start_time))
                        contourCheck = 0
                        # play a sound
        #                soundhandle = SoundClient()
                        rospy.sleep(0.1)
        #                soundhandle.stopAll()
        #                soundhandle.play(SoundRequest.NEEDS_UNPLUGGING)
                        rospy.sleep(0.2)
                        # save the map
                        cv2.imwrite('mazemap.png',odata)
                        rospy.loginfo("Reached after write")
                    else:
                        rotatebot(10)
            rospy.loginfo("Reached check")
            rospy.loginfo(closure(odata))
            if closure(odata):
                # map is complete, so save current time into file
                rospy.loginfo("Closure activated")
                with open("maptime.txt", "w") as f:
                    f.write("Elapsed Time: " + str(time.time() - start_time))
                contourCheck = 0
                # play a sound
#                soundhandle = SoundClient()
                rospy.sleep(0.1)
#                soundhandle.stopAll()
#                soundhandle.play(SoundRequest.NEEDS_UNPLUGGING)
                rospy.sleep(0.2)
                # save the map
                cv2.imwrite('mazemap.png',odata)
                rospy.loginfo("Reached after write")

#            time.sleep(1)
            
        rospy.loginfo("COme TO paoa")
        twist.linear.x=0
        twist.angular.z=rotate_speed
        time.sleep(0.1)
        pub.publish(twist)
        time.sleep(5)
        
    except KeyboardInterrupt:
        pass
    

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
