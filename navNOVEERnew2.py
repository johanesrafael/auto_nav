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
import cv2
import tf2_ros

laser_range = np.array([])
occdata = []
odata=[]
yaw = 0.0
t_yaw=0.0
rotate_speed = 0.20
linear_speed = 0.1
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


def get_occupancy(msg, tfBuffer):
    global occdata
    global odata

    # create numpy array
    data1 = np.array([msg.data])
    # compute histogram to identify percent of bins with -1
    occ_counts = np.histogram(data1,occ_bins)
    # calculate total number of bins
    total_bins = msg.info.width * msg.info.height
    
    oc2 = data1 + 1
    odata = np.uint8(oc2.reshape(msg.info.height,msg.info.width,order='F'))

    # set all values above 1 (i.e. above 0 in the original map data, representing occupied locations)
    oc3 = (oc2>1).choose(oc2,2)
    # reshape to 2D array using column order
    
#    occdata = np.uint8(oc3.reshape(msg.info.height,msg.info.width,order='F'))
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
    ALTHRESH = 8
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
    global t_yaw
    global yaw
    global odata
    
    rospy.init_node("move", anonymous=True)
    
    pub=rospy.Publisher("cmd_vel",Twist,queue_size=10)
    
    # subscribe to odometry data
    rospy.Subscriber('odom', Odometry, get_odom_dir)
    # subscribe to LaserScan data
    rospy.Subscriber('scan', LaserScan, get_laserscan)
    # subscribe to map occupancy data
#    rospy.Subscriber('map', OccupancyGrid, get_occupancy)
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    rospy.sleep(1.0)

    # subscribe to map occupancy data
    rospy.Subscriber('map', OccupancyGrid, get_occupancy, tfBuffer)
    
    rate=rospy.Rate(1000)
    
    rate.sleep()
    time.sleep(1)
    
    t_yaw=math.degrees(yaw)
    
    findWall()
    twist=Twist()
    
    sideD=np.amin(laser_range[0,range(87,94,1)])
    frontD=np.amin(laser_range[0,range(-3,4,1)])
    
    counter=0
    
    contourCheck=1
    start_time = time.time()
    
    try:
        while contourCheck==1:
#            twist.linear.x=linear_speed
#            twist.angular.x=0
#            time.sleep(0.05)
#            pub.publish(twist)
        
            counter+=1
            
#            rospy.loginfo(str(sideD) + " " + str(np.amin(laser_range[0,range(87,94,1)])))
            
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
                time.sleep(1.20) #0.2
                pub.publish(twist)
                rotatebot(t_yaw+90)
                t_yaw+=90
                twist.linear.x=linear_speed
                time.sleep(0.1)
                pub.publish(twist)
                time.sleep((sideD/linear_speed)*1.4)
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
                #rotatebot(t_yaw)
                counter=0
            if closure(odata)==True:
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
        
        rospy.loginfo("COme to Papa")
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
    
