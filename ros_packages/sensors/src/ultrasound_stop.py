#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Range



def sensor_data(range):
    pass


if __name__ == '__main__':
    try:
        rospy.init_node('sonar_stop')

        rospy.Subscriber('ultrasound_front', Range, sensor_data)
        rospy.Subscriber('ultrasound_rear', Range, sensor_data)
        rospy.Subscriber('ultrasound_left', Range, sensor_data)
        rospy.Subscriber('ultrasound_right', Range, sensor_data)

        encoder_pub = rospy.Publisher('obstacle/stop', Bool, queue_size=50)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
