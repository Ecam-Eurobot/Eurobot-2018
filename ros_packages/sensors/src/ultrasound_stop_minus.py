#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Range
from move_base_msgs.msg import MoveBaseActionFeedback

obstacle = False
MAX_DISTANCE = 0.2
encoder_pub = None

position = None
initial = None
# Robot
robot = rospy.get_param("/team")
if robot == "green":
    initial = {"x": 0.4, "y": -0.65}
else:
    initial = {"x": 2.6, "y": -0.65}


def sensor_data(sensor):
    global obstacle, initial, position, encoder_pub
    if not obstacle and sensor.range <= MAX_DISTANCE and sensor.range != 0:
        if robot == "green" and position.x > initial['x'] and position.y < initial['y']:
            obstacle = True
            encoder_pub.publish(True)
        elif robot == "red" and position.x < initial['x'] and position.y < initial['y']:
            obstacle = True
            encoder_pub.publish(True)
    elif obstacle and (sensor.range > 0.2 or sensor.range == 0):
        obstacle = False
        encoder_pub.publish(False)


def save_pos(fb):
    global position
    rospy.loginfo(fb)
    #position = fb.base_position.pose.position

if __name__ == '__main__':
    try:
        rospy.init_node('sonar_stop')

        rospy.Subscriber('ultrasound_front_left', Range, sensor_data)
        rospy.Subscriber('ultrasound_front_right', Range, sensor_data)
        rospy.Subscriber('ultrasound_rear', Range, sensor_data)
        rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, save_pos)
        encoder_pub = rospy.Publisher('obstacle/stop', Bool, queue_size=50)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
