#!/usr/bin/env python

import rospy
from tf.transformations import *
import actionlib
from actionlib_msgs.msg import *
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt


class MoveBaseTest:
    def __init__(self):
        self.initial_pose = 0
        self.rest_time = 5
        rospy.init_node('minus_nav')
        rospy.on_shutdown(self.shutdown)

        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED', 'LOST']

        obj = Pose(Point(2.50, -0.5, 0.0), Quaternion(quaternion_from_euler(0.0, 0.0, 3.14159, axes='sxyz')))

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        # Wait 60 seconds for the action server to become available
        wait = self.move_base.wait_for_server(rospy.Duration(60))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available...")

        rospy.loginfo("Connected to move base server")

        # A variable to hold the initial pose of the robot to be set by
        # the user in RViz
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.pose = Pose(Point(1.00, -1.00, 0.0), Quaternion(quaternion_from_euler(0.0, 0.0, 3.14159, axes='sxyz')))

        start = rospy.Publisher('start', String, queue_size=2)
        rospy.sleep(3)
        start.publish("Start")
        pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
        pub.publish(initial_pose)

        rospy.loginfo("Starting navigation test")
        # Set up the next goal location
        self.goal = MoveBaseGoal()
        self.goal.target_pose.pose = obj
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()

        # Let the user know where the robot is going next
        rospy.loginfo("Going to: " + str(obj))

        # Start the robot toward the next location
        self.move_base.send_goal(self.goal)

        # Allow 5 minutes to get there
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))

        # Check for success or failure
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")
                rospy.loginfo("State:" + str(state))
            else:
                rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))

        rospy.sleep(self.rest_time)

    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(1)


def trunc(f, n):
    # Truncates/pads a float f to n decimal places without rounding
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])


if __name__ == '__main__':
    try:
        MoveBaseTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation test finished.")
