#!/usr/bin/env python

import sys
import random
import rospy
import tf
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovarianceStamped
from time import sleep
raspberry = "-r" in sys.argv

# If passed the -r argument, load the rapsberry libs
if raspberry:
    import RPi.GPIO as GPIO

x = 0.0
y = 0.0
th = 0.0

vx = 0.0
vy = 0.0
vth = 0.0

time = None

rospy.init_node('startup_conf')

reset_pub = rospy.Publisher('reset', Empty, queue_size=1)
odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()
#
#   GPIO PIN CONFIGURATIONS
#
# This pin will be used to configure the team
pin_team = rospy.get_param("/pins/team_switch")

# This pin will be used to drive a led to indicate that the team has been set correctly in ROS
pin_team_feedback = rospy.get_param("/pins/team_feedback_led")

# This pin will be used to reset the initial position with the current configuration
pin_reset = rospy.get_param("/pins/reset_button")

def broadcast_transform(x, y, th, time):
    # since all odometry is 6DOF we'll need a quaternion created from yaw
    rotation = tf.transformations.quaternion_from_euler(0, 0, th)

    odom_broadcaster.sendTransform(
        (x, y, 0),
        rotation,
        time,
        "base_link",
        "odom"
    )


def publish_odom(x, y, th, vx, vy, vth, time):
    odom_msg = Odometry()

    odom_msg.header.stamp = time
    odom_msg.header.frame_id = "odom"

    # set the position
    odom_msg.pose.pose = Pose(Point(x, y, 0.), Quaternion(*tf.transformations.quaternion_from_euler(0, 0, th)))

    # set the velocity
    odom_msg.child_frame_id = "base_link"
    odom_msg.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    # publish the message
    sleep(0.1)
    odom_pub.publish(odom_msg)

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
            pos = rospy.get_param("/start/green/position")

            x = pos['x']
            y = pos['y']
            th = pos['orientation']
            time = rospy.Time.now()

            broadcast_transform(x, y, th, time)
            publish_odom(x, y, th, 0, 0, 0, time)
        else:
            rospy.set_param("/team", "red")
            rospy.set_param("/reset/position", rospy.get_param("/start/red/position"))
            pos = rospy.get_param("/start/red/position")

            x = pos['x']
            y = pos['y']
            th = pos['orientation']
            time = rospy.Time.now()

            broadcast_transform(x, y, th, time)
            publish_odom(x, y, th, 0, 0, 0, time)
    else:
        team = rospy.get_param("/team")
        if team == "green":
            rospy.set_param("/reset/position", rospy.get_param("/start/green/position"))
            pos = rospy.get_param("/start/green/position")

            x = pos['x']
            y = pos['y']
            th = pos['orientation']
            time = rospy.Time.now()

            broadcast_transform(x, y, th, time)
            publish_odom(x, y, th, 0, 0, 0, time)

        elif team == "red":
            rospy.set_param("/reset/position", rospy.get_param("/start/red/position"))
            pos = rospy.get_param("/start/red/position")

            x = pos['x']
            y = pos['y']
            th = pos['orientation']
            time = rospy.Time.now()

            broadcast_transform(x, y, th, time)
            publish_odom(x, y, th, 0, 0, 0, time)
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


