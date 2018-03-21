#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from ecam_msg.msg import Encoders

from math import cos, sin, pi

ticks_front_left = 0
ticks_front_right = 0
ticks_rear_left = 0
ticks_rear_right = 0

position_front_left = 0
position_front_right = 0
position_rear_left = 0
position_rear_right = 0

time = None


def publish_joint_state(encoder):
    global ticks_front_left, ticks_front_right, ticks_rear_left, ticks_rear_right
    global position_front_left, position_front_right, position_rear_left, position_rear_right
    global time

    wheels_state = JointState()
    wheels_state.header.stamp = rospy.Time.now()

    wheels_state.name = [
        'base_link_to_wheel_front_left',
        'base_link_to_wheel_front_right',
        'base_link_to_wheel_rear_left',
        'base_link_to_wheel_rear_right'
    ]

    new_time = rospy.Time.now()

    new_ticks_front_left = encoder.front_left
    new_ticks_front_right = encoder.front_right
    new_ticks_rear_left = encoder.rear_left
    new_ticks_rear_right = encoder.rear_right

    # Compute joint velocities
    v_front_left = compute_velocity(new_ticks_front_left, ticks_front_left, position_front_left, new_time)
    v_front_right = compute_velocity(new_ticks_front_right, ticks_front_right, position_front_right, new_time)
    v_rear_left = compute_velocity(new_ticks_rear_left, ticks_rear_left, position_rear_left, new_time)
    v_rear_right = compute_velocity(new_ticks_rear_right, ticks_rear_right, position_rear_right, new_time)

    wheels_state.velocity = [
        v_front_left,
        v_front_right,
        v_rear_left,
        v_rear_right
    ]

    # Compute joint positions
    position_front_left += compute_distance(new_ticks_front_left, ticks_front_left)
    position_front_right += compute_distance(new_ticks_front_right, ticks_front_right)
    position_rear_left += compute_distance(new_ticks_rear_left, ticks_rear_left)
    position_rear_right += compute_distance(new_ticks_rear_right, ticks_rear_right)

    wheels_state.position = [
        position_front_left,
        position_front_right,
        position_rear_left,
        position_rear_right
    ]

    # Update saved tick values
    ticks_front_left = new_ticks_front_left
    ticks_front_right = new_ticks_front_right
    ticks_rear_left = new_ticks_rear_left
    ticks_rear_right = new_ticks_rear_right
    time = new_time

    joint_states_pub.publish(wheels_state)


def compute_distance(new_ticks, old_ticks):
    delta = new_ticks - old_ticks
    return float(delta) / ENCODER_TICKS_PER_REV * 2 * pi

def compute_velocity(new_ticks, old_ticks, old_position, new_time):
    global time
    new_position = old_position + compute_distance(new_ticks, old_ticks)
    return (new_position - old_position) / (new_time - time).to_sec()



if __name__ == '__main__':
    try:
        rospy.init_node('encoder_to_jointstate')

        ENCODER_TICKS_PER_REV = rospy.get_param("/encoder/ticks_per_rev")

        encoder_sub = rospy.Subscriber('encoder', Encoders, publish_joint_state)

        joint_states_pub = rospy.Publisher('joint_states', JointState, queue_size=50)

        time = rospy.Time.now()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
