#!/usr/bin/env python

import RPi.GPIO as GPIO

import rospy
from std_msgs.msg import Empty


rospy.init_node('startup_conf')
reset_pub = rospy.Publisher('initialpose/reset', Empty, queue_size=1)

#
#   GPIO PIN CONFIGURATIONS
#

GPIO.setmode(GPIO.BCM)

# This pin will be used to configure the team
pin_team = rospy.get_param("/pins/team_switch")

# This pin will be used to drive a led to indicate that the team has been set correctly in ROS
pin_team_feedback = rospy.get_param("/pins/team_feedback_led")

# This pin will be used to reset the initial position with the current configuration
pin_reset = rospy.get_param("/pins/reset_button")

GPIO.setup(pin_team, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(pin_team_feedback, GPIO.OUT)
GPIO.setup(pin_reset, GPIO.IN, pull_up_down=GPIO.PUD_UP)


def update_team(pin):
    if GPIO.input(pin_team):
        rospy.set_param("/team", "green")
    else:
        rospy.set_param("/team", "red")

def reset(pin):
    reset_pub.publish(Empty())


GPIO.add_event_detect(pin_team, GPIO.BOTH, callback=update_team)
GPIO.add_event_detect(pin_reset, GPIO.RISING, callback=reset)

# Update team on startupand reset
update_team(pin_team)
reset(pin_reset)


while not rospy.is_shutdown():
    pass


