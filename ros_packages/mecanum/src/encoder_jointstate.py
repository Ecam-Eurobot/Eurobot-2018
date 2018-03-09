#!/usr/bin/env python

import rospy
import tf
from std_msgs.msg import Int64
from sensor_msgs.msg import JointState

import math

encoder_position = None
old_encoder_ticks = 0
old_encoder_time = None
position = 0
ENCODER_TICKS_PER_REV = 3200
joint_states_pub = None

def wheel_joint_update(encoder):
    wheel_state = JointState()
    wheel_state.header.stamp = rospy.Time.now()

    wheel_state.name = ['base_link_to_wheel_'+ encoder_position.replace('/', '_')]

    global old_encoder_ticks
    global old_encoder_time
    global position
    global joint_states_pub

    delta = encoder.data - old_encoder_ticks
    old_encoder_ticks = encoder.data

    old_position = position
    #position += delta / ENCODER_TICKS_PER_REV * 2 * math.pi
    position += float(delta) / ENCODER_TICKS_PER_REV * 2 * math.pi

    time = rospy.Time.now()
    velocity = (position - old_position) / (old_encoder_time - time).to_sec()
    old_encoder_time = time

    wheel_state.position = [position]
    wheel_state.velocity = [velocity]

    joint_states_pub.publish(wheel_state)

if __name__ == '__main__':
    try:
        rospy.init_node('encoder_to_odom')

        encoder_position = rospy.get_param('~encoder/position')
        ENCODER_TICKS_PER_REV = rospy.get_param("~encoder/ticks_per_rev")

        front_left_sub = rospy.Subscriber('mecanum/encoder/' + encoder_position, Int64, wheel_joint_update)
        joint_states_pub = rospy.Publisher('joint_states', JointState, queue_size=50)

        old_encoder_time = rospy.Time.now()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
