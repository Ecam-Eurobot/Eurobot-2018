#!/usr/bin/env python

import random
import math

import rospy
import tf
import actionlib

from actionlib_msgs.msg import *
from std_msgs.msg import Empty, Int16, Int8, Bool
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

GOAL_STATES = ['PENDING', 'ACTIVE', 'PREEMPTED', 'SUCCEEDED', 'ABORTED', 'REJECTED',
               'PREEMPTING', 'RECALLING', 'RECALLED', 'LOST']


class Cortex:
    def __init__(self):
        rospy.init_node('cortex_strategy')

        rospy.on_shutdown(self.shutdown)

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        self.reset_requested = False
        self.started = False
        self.paused = False
        self.current_action = None
        self.goal = None

        self.actions = rospy.get_param("/actions/{}".format(rospy.get_param('team')))

        self.gun_pub = rospy.Publisher('gun_control', Int16, queue_size=10)
        self.water_purification_pub = rospy.Publisher('water_purification', Int16, queue_size=10)

        rospy.loginfo("Cortex waiting for move_base action server...")

        # Wait 60 seconds for the action server to become available
        wait = self.move_base.wait_for_server(rospy.Duration(30))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available...")

        rospy.loginfo("Cortex connected to move base server")

    def run(self, _):
        rospy.loginfo("Cortex running")
        rospy.loginfo(self.actions)
        rospy.logwarn("START")

        self.started = True

        for action in self.actions:
            action_done = False
            while not action_done:
                if self.paused:
                    rospy.sleep(0.1)
                    continue

                rospy.loginfo("Starting action: " + str(action))
                if action['type'] == "move":
                    x = action['position']['x']
                    y = action['position']['y']
                    th = action['position']['orientation']
                    if 'timeout' in action:
                        action_done = self.move_to(x, y, th, timeout=action['timeout'])
                    else:
                        action_done = self.move_to(x, y, th)
                    rospy.loginfo(action_done)
                elif action['type'] == "gun":
                    self.gun(action['value'])
                    action_done = True
                elif action['type'] == "water_purification":
                    self.water_purification(action['value'])
                    action_done = True
                elif action['type'] == "wait":
                    rospy.sleep(action['value'])
                    action_done = True
                else:
                    rospy.logerr("Action unknown: " + str(action))
                    action_done = True

            if self.reset_requested:
                self.reset_requested = False
                rospy.loginfo("Exiting action loop, reset requested")
                self.started = False
                return

        self.started = False

    def move_to(self, x, y, orientation, timeout=30):
        pose = Pose(Point(x, y, 0.), Quaternion(*tf.transformations.quaternion_from_euler(0, 0, orientation)))

        # Set up the next goal location
        self.goal = MoveBaseGoal()
        self.goal.target_pose.pose = pose
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()

        self.move_base.send_goal(self.goal)

        rospy.loginfo("Waiting for move_base to finish")
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(timeout))
        rospy.loginfo("move_base finished!")

        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
            return True
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded !")
                return True
            else:
                rospy.loginfo("Goal failed with error code: " + str(GOAL_STATES[state]))
                return False

    def gun(self, value):
        self.gun_pub.publish(value)

    def water_purification(self, value):
        self.water_purification_pub.publish(value)

    def reset(self, _):
        if self.started:
            self.reset_requested = True

        self.actions = rospy.get_param("/actions/{}".format(rospy.get_param('team')))
        self.move_base.cancel_goal()
        rospy.logwarn("RESET")

    def pause(self, value):
        self.paused = value.data
        rospy.loginfo(self.paused)
        if self.paused:
            rospy.logwarn("Paused action")
            self.move_base.cancel_goal()
        else:
            rospy.logwarn("Restarted action")


    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(0.2)


if __name__ == '__main__':
    try:
        cortex = Cortex()

        start_sub = rospy.Subscriber('start', Empty, cortex.run)
        reset_sub = rospy.Subscriber('reset', Empty, cortex.reset)
        reset_sub = rospy.Subscriber('obstacle/stop', Bool, cortex.pause)

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Cortex out! *mic drop*")