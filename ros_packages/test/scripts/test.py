#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

import math
import serial

class driver:
    def __init__(self):
        # init ros
        rospy.init_node('car_driver', anonymous=True)
        #rospy.Subscriber('/cmd_vel', Twist, self.get_cmd_vel)
        self.ser = serial.Serial('/dev/ttyACM0', 115200)
        self.get_arduino_message()

    # receive serial text from arduino and publish it to '/arduino' message
    def get_arduino_message(self):
        pub = rospy.Publisher('arduino', String, queue_size=10)
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            message = self.ser.readline()
            pub.publish(message)
            r.sleep()


if __name__ == '__main__':
    try:
        d = driver()
    except rospy.ROSInterruptException:
        pass
