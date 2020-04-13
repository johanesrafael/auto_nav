#!/usr/bin/env python
import time
import rospy
import RPi.GPIO as GPIO
import numpy as np
from sensor_msgs.msg import LaserScan


#pin1 = 20
pin2 = 26
GPIO.setmode(GPIO.BOARD)

#GPIO.setup(pin1,GPIO.OUT)
GPIO.setup(pin2,GPIO.OUT)
#p=GPIO.PWM(pin2,50)





laser_range = np.array([])


def get_laserscan(msg):
    global laser_range

    # create numpy array
    laser_range = np.array([msg.ranges])



def e2_2():
        global laser_range
        rospy.init_node('e2_2', anonymous=True)

        # subscribe to LaserScan data
        rospy.Subscriber('scan', LaserScan, get_laserscan)

        rate = rospy.Rate(1)

        rate.sleep()

        #p.start(2.5)

        time.sleep(1)

        print("Reached")
        while True:

                GPIO.output(pin2,GPIO.HIGH)
                print(1)
                time.sleep(1)
                GPIO.output(pin2,GPIO.LOW)
                print(2)
                time.sleep(1)
        #GPIO.output(pin2,GPIO.LOW)
        #time.sleep(.5)

                        #p.ChangeDutyCycle(2.5 + (float(45)/181)*10)
                        #time.sleep(.5)

                        #break

        #p.stop()
        GPIO.cleanup()


if __name__ == '__main__':
    try:
        e2_2()
    except rospy.ROSInterruptException:
        pass




