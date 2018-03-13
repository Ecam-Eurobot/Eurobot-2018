#!/usr/bin/env python

import rospy
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from std_msgs.msg import Empty

x = y = 0

rospy.init_node('mecanum')

initialpose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)

def reset(empty):
    # By default use green if the parameter is not set
    team = rospy.get_param("/team", "red")
    print("Team: " + team)
    coordinates = rospy.get_param("/initialpose/" + team + "/coordinates")
    orientation = rospy.get_param("/initialpose/" + team + "/orientation")

    initialpose_msg = PoseWithCovarianceStamped()
    initialpose_msg.header.stamp = rospy.get_rostime()
    initialpose_msg.pose.pose.position.x = coordinates['x']
    initialpose_msg.pose.pose.position.y = coordinates['y']
    initialpose_msg.pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, orientation))

    initialpose_pub.publish(initialpose_msg)

# Execute reset once on launch
reset(None)

rospy.Subscriber('initialpose/reset', Empty, reset)

try:
    rospy.spin()
except rospy.ROSInterruptException:
    pass


