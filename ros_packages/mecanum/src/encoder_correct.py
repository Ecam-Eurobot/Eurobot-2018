#!/usr/bin/env python

import rospy
from ecam_msg.msg import Encoders

def correct(encoders):
    #encoders.front_right = -encoders.front_right
    encoders.rear_right = -encoders.rear_right

    encoder_pub.publish(encoders)


if __name__ == '__main__':
    try:
        rospy.init_node('encoder_correct')

        encoder_sub = rospy.Subscriber('arduino/encoder', Encoders, correct)
        encoder_pub = rospy.Publisher('encoder', Encoders, queue_size=50)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
