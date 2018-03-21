#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Empty
from ecam_msg.msg import Encoders

from math import pi

# Current velocity for each wheel
v_fl = v_fr = v_rl = v_rr = 0

# Target velocity for each wheel
v_fl_target = v_fr_target = v_rl_target = v_rr_target = 0

# Encoders
encoders_msg = Encoders()



# Callbacks
def front_left_velocity_update(vel):
    global  v_fl_target
    v_fl_target = vel.data

def front_right_velocity_update(vel):
    global  v_fr_target
    v_fr_target = vel.data

def rear_left_velocity_update(vel):
    global  v_rl_target
    v_rl_target = vel.data

def rear_right_velocity_update(vel):
    global  v_rr_target
    v_rr_target = vel.data

def reset(empty):
    global  encoders_msg
    encoders_msg = Encoders()



rospy.init_node('motor_sim')

ENCODER_TICKS_PER_REV = rospy.get_param("/encoder/ticks_per_rev")

rospy.Subscriber('motor/front/left', Float32, front_left_velocity_update)
rospy.Subscriber('motor/front/right', Float32, front_right_velocity_update)
rospy.Subscriber('motor/rear/left', Float32, rear_left_velocity_update)
rospy.Subscriber('motor/rear/right', Float32, rear_right_velocity_update)
rospy.Subscriber('reset', Empty, reset)

encoder_pub = rospy.Publisher('encoder', Encoders, queue_size=50)

dt = 0.02

rate = rospy.Rate(1/dt)
while not rospy.is_shutdown():
    # Update speeds
    v_fl = v_fl_target
    v_fr = v_fr_target
    v_rl = v_rl_target
    v_rr = v_rr_target

    # Send encoder ticks
    encoders_msg.front_left += v_fl / (2 * pi) * ENCODER_TICKS_PER_REV * dt
    encoders_msg.front_right += v_fr / (2 * pi) * ENCODER_TICKS_PER_REV * dt
    encoders_msg.rear_left += v_rl / (2 * pi) * ENCODER_TICKS_PER_REV * dt
    encoders_msg.rear_right += v_rr / (2 * pi) * ENCODER_TICKS_PER_REV * dt

    encoder_pub.publish(encoders_msg)

    rate.sleep()