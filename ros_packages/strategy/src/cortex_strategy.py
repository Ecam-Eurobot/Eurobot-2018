#!/usr/bin/env python

import random
import math

import rospy
import tf
import actionlib

from actionlib_msgs.msg import *
from std_msgs.msg import Empty
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
        self.goal = None

        self.actions = rospy.get_param("/actions")

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
        for action in self.actions:
            if action['type'] == "move":
                x = action['position']['x']
                y = action['position']['y']
                th = action['position']['orientation']
                self.move_to(x, y, th)
            else:
                rospy.logerr("Action unknown: " + str(action))

            if self.reset_requested:
                self.reset_requested = False
                return

    def move_to(self, x, y, orientation, timeout=30):
        pose = Pose(Point(x, y, 0.), Quaternion(*tf.transformations.quaternion_from_euler(0, 0, orientation)))

        # Set up the next goal location
        self.goal = MoveBaseGoal()
        self.goal.target_pose.pose = pose
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()

        self.move_base.send_goal(self.goal)

        finished_within_time = self.move_base.wait_for_result(rospy.Duration(timeout))

        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                return True
            else:
                rospy.loginfo("Goal failed with error code: " + str(GOAL_STATES[state]))
                return False

    def reset(self, _):
        self.reset_requested = True
        self.move_base.cancel_goal()

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(0.2)


if __name__ == '__main__':
    try:
        cortex = Cortex()

        start_sub = rospy.Subscriber('start', Empty, cortex.run)
        reset_sub = rospy.Subscriber('reset', Empty, cortex.reset)

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Cortex out! *mic drop*")