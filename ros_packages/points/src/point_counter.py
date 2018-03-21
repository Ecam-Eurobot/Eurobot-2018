#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import Int16, Empty
from ecam_msg.msg import Tower
from points import Points

# Object storing the points
points = Points()

rospy.init_node('points_manager')
points_pub = rospy.Publisher('PointDisplay', Int16, queue_size=10)


def recuperator(id):
    global points
    points.emptied_recuperator(id.data)
    publish_points()

def watertower(n):
    global points
    points.balls_shot_in_watertower(n.data)
    publish_points()

def treatment_plant(empty):
    global points
    points.balls_in_treatment_plant()
    publish_points()

def tower(t):
    global points
    print(t)
    points.placed_tower(t.height, t.sequence)
    publish_points()

def bee(empty):
    global points
    points.bee_popped()
    publish_points()

def panel(empty):
    global points
    points.panel_powered()
    publish_points()

def start(empty):
    publish_points()

def publish_points():
    global points_pub, points
    msg = Int16()
    msg.data = points.total_points()
    points_pub.publish(msg)


# Topics from which point notifications come
rospy.Subscriber('points/recuperator', Int16, recuperator)
rospy.Subscriber('points/watertower', Int16, watertower)
rospy.Subscriber('points/treatment_plant', Empty, treatment_plant)
rospy.Subscriber('points/tower', Tower, tower)
rospy.Subscriber('points/bee', Empty, bee)
rospy.Subscriber('points/panel', Empty, panel)

rospy.Subscriber('start', Empty, start)

try:
    time.sleep(0.1)
    points_pub.publish(Int16(0))
    rospy.spin()
except rospy.ROSInterruptException:
    pass
