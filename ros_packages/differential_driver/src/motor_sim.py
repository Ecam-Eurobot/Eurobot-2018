#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Empty

from math import pi

# Current velocity for each wheel
v_l = v_r = 0

# Target velocity for each wheel
v_l_target = v_r_target = 0

# Encoders
left = right = 0

# Callbacks
def left_velocity_update(vel):
    global  v_l_target
    v_l_target = vel.data

def right_velocity_update(vel):
    global  v_r_target
    v_r_target = vel.data

def reset(empty):
    global  encoders_msg
    encoders_msg = Encoders()



rospy.init_node('motor_sim_minus')

ENCODER_TICKS_PER_REV = 4100

rospy.Subscriber('lmotor_cmd', Float32, left_velocity_update)
rospy.Subscriber('rmotor_cmd', Float32, right_velocity_update)
rospy.Subscriber('initialpose/reset', Empty, reset)

encoder_pub_r = rospy.Publisher('rwheel', Float32, queue_size=50)
encoder_pub_l = rospy.Publisher('lwheel', Float32, queue_size=50)

dt = 0.1

rate = rospy.Rate(1/dt)
while not rospy.is_shutdown():
    # Update speeds
    v_l = v_l_target
    v_r = v_r_target

    # Send encoder ticks
    left += v_l / (2 * pi) * ENCODER_TICKS_PER_REV * dt
    right += v_r / (2 * pi) * ENCODER_TICKS_PER_REV * dt

    encoder_pub_r.publish(right)
    encoder_pub_l.publish(left)

    rate.sleep()
