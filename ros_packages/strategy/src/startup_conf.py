#!/usr/bin/env python

import sys
import random
import rospy
from std_msgs.msg import Empty

raspberry = "-r" in sys.argv

# If passed the -r argument, load the rapsberry libs
if raspberry:
    import RPi.GPIO as GPIO

rospy.init_node('startup_conf')

reset_pub = rospy.Publisher('reset', Empty, queue_size=1)

#
#   GPIO PIN CONFIGURATIONS
#
# This pin will be used to configure the team
pin_team = rospy.get_param("/pins/team_switch")

# This pin will be used to drive a led to indicate that the team has been set correctly in ROS
pin_team_feedback = rospy.get_param("/pins/team_feedback_led")

# This pin will be used to reset the initial position with the current configuration
pin_reset = rospy.get_param("/pins/reset_button")

if raspberry:
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(pin_team, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(pin_team_feedback, GPIO.OUT)
    GPIO.setup(pin_reset, GPIO.IN, pull_up_down=GPIO.PUD_UP)


def update_team(pin):
    if raspberry:
        if GPIO.input(pin_team):
            rospy.set_param("/team", "green")
            rospy.set_param("/reset/position", rospy.get_param("/start/green/position"))
        else:
            rospy.set_param("/team", "red")
            rospy.set_param("/reset/position", rospy.get_param("/start/red/position"))
    else:
        team = rospy.get_param("/team")
        if team == "green":
            rospy.set_param("/reset/position", rospy.get_param("/start/green/position"))
        elif team == "red":
            rospy.set_param("/reset/position", rospy.get_param("/start/red/position"))
        else:
            rospy.set_param("/team", random.choice(["green", "red"]))
            update_team(pin_team)

    # Call reset to reset the coordinates on toggle switch change
    reset(pin)

def reset(pin):
    reset_pub.publish(Empty())


if raspberry:
    GPIO.add_event_detect(pin_team, GPIO.BOTH, callback=update_team)
    GPIO.add_event_detect(pin_reset, GPIO.RISING, callback=reset)

# Update team on startup
update_team(pin_team)


while not rospy.is_shutdown():
    pass


