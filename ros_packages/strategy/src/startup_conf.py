#!/usr/bin/env python

import sys
import random
import rospy
import pigpio
from std_msgs.msg import Empty

pi = pigpio.pi()

rospy.init_node('startup_conf')

# Robot
robot = rospy.get_param("/robot")

pub_start = rospy.Publisher('start', Empty, queue_size=1)
pub_reset = rospy.Publisher('reset', Empty, queue_size=1)


#
#   GPIO PIN CONFIGURATIONS
#


µ
# This pin will be used to configure the team
pin_team = rospy.get_param("/pins/team_switch")

# This pin will be used to drive a led to indicate that the team has been set correctly in ROS
pin_team_feedback = rospy.get_param("/pins/team_feedback_led")

# This pin will be used to drive a led to indicate that the strategy has been set correctly in ROS
pin_strategy_feedback = rospy.get_param("/pins/strategy_feedback_led")

# This pin will be used to launch robot
pin_start = rospy.get_param("/pins/start")




def update_team(gpio, level, tick):
    if level:
        rospy.set_param("/team", "red")
        rospy.set_param("/reset/position", rospy.get_param("/start/{}/red/position".format(robot)))
        pi.write(pin_team_feedback, 1)
    else :
        rospy.set_param("/team", "green")
        rospy.set_param("/reset/position", rospy.get_param("/start/{}/green/position".format(robot)))
        # Blink 2 times for acknowledge
        pi.write(pin_team_feedback, 0)
    reset()




publish = True
def start(gpio, level, tick):
    global publish
    rospy.sleep(0.2)
    if not pi.read(pin_start):
        if publish :
            pi.write(pin_strategy_feedback, 1)
            pub_start.publish(Empty())
            publish = False


def reset():
    pub_reset.publish(Empty())
    if pi.read(pin_start):
        global publish
        publish = True
        pi.write(pin_strategy_feedback, 0)



com = False
init = True

if init:
    pi.set_mode(pin_team, pigpio.INPUT)
    pi.set_mode(pin_start, pigpio.INPUT)
    pi.set_mode(pin_team_feedback, pigpio.OUTPUT)
    pi.set_mode(pin_strategy_feedback, pigpio.OUTPUT)
    pi.set_pull_up_down(pin_start, pigpio.PUD_DOWN)
    pi.set_pull_up_down(pin_strategy_feedback, pigpio.PUD_DOWN)
    teamInterrupt = pi.callback(pin_team, pigpio.EITHER_EDGE, update_team)
    startInterrupt = pi.callback(pin_start, pigpio.EITHER_EDGE, start)

while not rospy.is_shutdown():
    pass