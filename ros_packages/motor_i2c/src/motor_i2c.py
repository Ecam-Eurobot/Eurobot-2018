#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

from i2c import I2C

pub = None

def motor_front_left(angular_velocity):
    # Tranform the value into a list of bytes
    data_bytes = list(bytearray(struct.pack('f', angular_velocity.data)))

    # Insert the register the motor controller uses for the angular velocity
    data_bytes.insert(0, 0x10)
    robot_i2c.send(data_bytes)

if __name__ == '__main__':
    try:
        rospy.init_node('motor_i2c')

        # Get parameters
        motor_position = rospy.get_param('~motor')
        i2c_address = rospy.get_param('motor/' + motor_position + '/i2c_address')

        # Init i2c bus
        robot_i2c = I2C(i2c_address)
        rospy.Subscriber('motor', Float32, motor_i2c)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
