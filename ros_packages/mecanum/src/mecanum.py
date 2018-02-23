#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

import math

WHEEL_SEPARATION_WIDTH = None
WHEEL_SEPARATION_LENGTH = None
WHEEL_GEOMETRY = None
WHEEL_RADIUS = None

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

        # Get parameters about the geometry of the wheels
        WHEEL_SEPARATION_WIDTH = rospy.get_param("wheel/separation/horizontal")
        WHEEL_SEPARATION_LENGTH = rospy.get_param("wheel/separation/vertical")
        WHEEL_GEOMETRY = (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) / 2
        WHEEL_RADIUS = rospy.get_param("wheel/diameter") / 2


        pub_mfl = rospy.Publisher('motor/front/left', Float32, queue_size=1)
        pub_mfr = rospy.Publisher('motor/front/right', Float32, queue_size=1)
        pub_mbl = rospy.Publisher('motor/rear/left', Float32, queue_size=1)
        pub_mbr = rospy.Publisher('motor/rear/right', Float32, queue_size=1)

        sub = rospy.Subscriber('cmd_vel', Twist, convert)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
