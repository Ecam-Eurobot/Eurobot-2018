#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

import math

WHEEL_SEPARATION_WIDTH = 0.150 / 2
WHEEL_SEPARATION_LENGTH = 0.250 / 2
WHEEL_GEOMETRY = WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH
WHEEL_RADIUS = 0.3

pub_mfl = None
pub_mfr = None
pub_mbl = None
pub_mbr = None

def convert(move):
    x = move.linear.x
    y = move.linear.y
    rot = move.angular.z

    front_right = - (y - x + rot * WHEEL_GEOMETRY) / WHEEL_RADIUS
    back_right = - (y + x + rot * WHEEL_GEOMETRY) / WHEEL_RADIUS
    back_left = (y - x + rot * WHEEL_GEOMETRY) / WHEEL_RADIUS
    front_left = (y + x + rot * WHEEL_GEOMETRY) / WHEEL_RADIUS

    pub_mfl.publish(front_left)
    pub_mfr.publish(front_right)
    pub_mbl.publish(back_left)
    pub_mbr.publish(back_right)


if __name__ == '__main__':
    try:
        rospy.init_node('mecanum')

        pub_mfl = rospy.Publisher('i2c/motor/fl', Float32, queue_size=1)
        pub_mfr = rospy.Publisher('i2c/motor/fr', Float32, queue_size=1)
        pub_mbl = rospy.Publisher('i2c/motor/bl', Float32, queue_size=1)
        pub_mbr = rospy.Publisher('i2c/motor/br', Float32, queue_size=1)

        sub = rospy.Subscriber('cmd_vel', Twist, convert)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass