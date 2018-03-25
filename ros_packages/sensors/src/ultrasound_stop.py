#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Range

obstacle = False

def sensor_data(reading):
    global obstacle

    if not obstacle and reading.range < 0.2 and reading.range != 0:
        obstacle_pub.publish(True)
        obstacle = True
    elif obstacle and (reading.range > 0.2 or reading.range == 0):
        obstacle_pub.publish(False)
        obstacle = False

def sensor_data_front_ignore(reading):
    global obstacle
    # Completely ignore, it is too close to the border
    if rospy.get_param("team") == "red":
        pass
    else:

        if not obstacle and reading.range < 0.2 and reading.range != 0:
            obstacle_pub.publish(True)
            obstacle = True
        elif obstacle and (reading.range > 0.2 or reading.range == 0):
            obstacle_pub.publish(False)
            obstacle = False

def sensor_data_rear_ignore(reading):
    global obstacle
    # Completely ignore, it is too close to the border
    if rospy.get_param("team") == "green":
        pass
    else:

        if not obstacle and reading.range < 0.2 and reading.range != 0:
            obstacle_pub.publish(True)
            obstacle = True
        elif obstacle and (reading.range > 0.2 or reading.range == 0):
            obstacle_pub.publish(False)
            obstacle = False


if __name__ == '__main__':
    try:
        rospy.init_node('sonar_stop')

        rospy.Subscriber('ultrasound_front', Range, sensor_data_front_ignore)
        rospy.Subscriber('ultrasound_rear', Range, sensor_data_rear_ignore)
        rospy.Subscriber('ultrasound_left', Range, sensor_data)
        rospy.Subscriber('ultrasound_right', Range, sensor_data)

        obstacle_pub = rospy.Publisher('obstacle/stop', Bool, queue_size=50)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
