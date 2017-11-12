#!/usr/bin/env python

import rospy
import tf
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

        locations = dict()

        locations['first'] = Pose(Point(1.50, -0.5, 0.0), Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0)))
        #locations['second'] = Pose(Point(1.50, -1.5, 0.0), Quaternion(0.000, 0.000, 0.223, 0.975))
        #locations['third'] = Pose(Point(1.00, -1.5, 0.0), Quaternion(0.000, 0.000, 0.223, 0.975))
        #locations['fourth'] = Pose(Point(0.50, -0.5, 0.0), Quaternion(0.000, 0.000, 0.223, 0.975))

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move base server")

        # A variable to hold the initial pose of the robot to be set by
        # the user in RViz
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.pose.pose.position.x = 1.0
        initial_pose.pose.pose.position.y = -1.0
        initial_pose.pose.pose.position.z = 0.0

        initial_pose.pose.pose.orientation.x = 0.0
        initial_pose.pose.pose.orientation.y = 0.0
        initial_pose.pose.pose.orientation.z = 0.0
        initial_pose.pose.pose.orientation.w = 0.0
        # Variables to keep track of success rate, running time,
        # and distance traveled
        n_locations = len(locations)
        n_goals = 0
        n_successes = 0
        i = n_locations
        distance_traveled = 0
        start_time = rospy.Time.now()
        running_time = 0
        location = ""
        last_location = ""

        # Get the initial pose from the user
        #rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
        #rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        self.last_location = Pose()
        start = rospy.Publisher('start', String, queue_size=2)
        rospy.sleep(3)
        start.publish("Start")
        pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
        pub.publish(initial_pose)

        # Make sure we have the initial pose
        #while initial_pose.header.stamp == "":
            #rospy.sleep(1)

        rospy.loginfo("Starting navigation test")
        #rospy.sleep(5)
        # Begin the main loop and run through a sequence of locations
        while not rospy.is_shutdown():
            # If we've gone through the current sequence,
            # start with a new random sequence
            if i == n_locations:
                i = 0
                sequence = sample(locations, n_locations)
                # Skip over first location if it is the same as
                # the last location
                if sequence[0] == last_location:
                    i = 1

            # Get the next location in the current sequence
            location = sequence[i]

            # Keep track of the distance traveled.
            # Use updated initial pose if available.
            if initial_pose.header.stamp == "":
                distance = sqrt(pow(locations[location].position.x -
                                    locations[last_location].position.x, 2) +
                                pow(locations[location].position.y -
                                    locations[last_location].position.y, 2))
            else:
                rospy.loginfo("Updating current pose.")
                distance = sqrt(pow(locations[location].position.x -
                                    initial_pose.pose.pose.position.x, 2) +
                                pow(locations[location].position.y -
                                    initial_pose.pose.pose.position.y, 2))
                initial_pose.header.stamp = ""

            # Store the last location for distance calculations
            last_location = location

            # Increment the counters
            i += 1
            n_goals += 1

            # Set up the next goal location
            self.goal = MoveBaseGoal()
            self.goal.target_pose.pose = locations[location]
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()

            # Let the user know where the robot is going next
            rospy.loginfo("Going to: " + str(location))

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
                    n_successes += 1
                    distance_traveled += distance
                    rospy.loginfo("State:" + str(state))
                else:
                    rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))

            # How long have we been running?
            running_time = rospy.Time.now() - start_time
            running_time = running_time.secs / 60.0

            # Print a summary success/failure, distance traveled and time elapsed
            rospy.loginfo("Success so far: " + str(n_successes) + "/" +
                          str(n_goals) + " = " +
                          str(100 * n_successes / n_goals) + "%")
            rospy.loginfo("Running time: " + str(trunc(running_time, 1)) +
                          " min Distance: " + str(trunc(distance_traveled, 1)) + " m")
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
