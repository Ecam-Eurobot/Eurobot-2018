#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from i2c import I2C

pub = None

def move_forward():
    try:
        robot_i2c.send(1)
    except IOError:
        print("I2C not connected...")
    pub.publish("0x01")

def stop():
    try:
        robot_i2c.send(6)
    except IOError:
        print("I2C not connected...")
    pub.publish("0x06")

def send_command(move):
    print(move)
    if move.linear.x > 0:
        move_forward()
    else:
        stop()    

if __name__ == '__main__':
    try:
        robot_i2c = I2C(0x05)
        rospy.init_node('twist_to_i2c')
        pub = rospy.Publisher('i2c', String, queue_size=1)
        sub = rospy.Subscriber('cmd_vel', Twist, send_command)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
