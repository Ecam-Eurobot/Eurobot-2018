#!/usr/bin/env python

from time import sleep
from math import cos, sin, pi

import rospy
import tf
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovarianceStamped
from ecam_msg.msg import Encoders


# If the delta of the encoder ticks is above this value,
# reject the value and ignore it in the odometry computation
REJECTION_THRESHOLD = 1000

ticks_front_left = None
ticks_front_right = None
ticks_rear_left = None
ticks_rear_right = None

x = 0.0
y = 0.0
th = 0.0

vx = 0.0
vy = 0.0
vth = 0.0

time = None


def update(encoder):
    global vx, vy, vth
    global x, y, th
    global time

    new_time = rospy.Time.now()

    v_front_left, v_front_right, v_rear_left, v_rear_right = velocities(encoder, new_time)

    # From wheel velocities, compute base velocity by using the inverse kinematics
    vx = (v_front_left + v_front_right + v_rear_left + v_rear_right) * WHEEL_RADIUS / 4
    vy = (-v_front_left + v_front_right + v_rear_left - v_rear_right) * WHEEL_RADIUS / 4
    vth = (-v_front_left + v_front_right - v_rear_left + v_rear_right) * WHEEL_RADIUS / (4 * (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH))

    dt = (new_time - time).to_sec()
    delta_x = (vx * cos(th) - vy * sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th)) * dt
    delta_th = vth * dt

    x += delta_x
    y += delta_y
    th += delta_th

    # print(x, y, th, vx, vy, vth)

    broadcast_transform(x, y, th, new_time)
    publish_odom(x, y, th, vx, vy, vth, new_time)

    time = new_time



def velocities(encoder, new_time):
    global ticks_front_left, ticks_front_right, ticks_rear_left, ticks_rear_right
    global time

    if None in [ticks_front_left, ticks_front_right, ticks_rear_left, ticks_rear_right]:
        ticks_front_left = encoder.front_left
        ticks_front_right = encoder.front_right
        ticks_rear_left = encoder.rear_left
        ticks_rear_right = encoder.rear_right

    new_ticks_front_left = reject_faulty_encoder_value(encoder.front_left, ticks_front_left)
    new_ticks_front_right = reject_faulty_encoder_value(encoder.front_right, ticks_front_right)
    new_ticks_rear_left = reject_faulty_encoder_value(encoder.rear_left, ticks_rear_left)
    new_ticks_rear_right = reject_faulty_encoder_value(encoder.rear_right, ticks_rear_right)

    v_front_left = compute_velocity(new_ticks_front_left, ticks_front_left, new_time)
    v_front_right = compute_velocity(new_ticks_front_right, ticks_front_right, new_time)
    v_rear_left = compute_velocity(new_ticks_rear_left, ticks_rear_left, new_time)
    v_rear_right = compute_velocity(new_ticks_rear_right, ticks_rear_right, new_time)

    ticks_front_left = new_ticks_front_left
    ticks_front_right = new_ticks_front_right
    ticks_rear_left = new_ticks_rear_left
    ticks_rear_right = new_ticks_rear_right

    return v_front_left, v_front_right, v_rear_left, v_rear_right



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
    odom_pub.publish(odom_msg)



def reject_faulty_encoder_value(new, old):
    if abs(new - old) > REJECTION_THRESHOLD:
        rospy.logwarn("Rejected encoder value (old: " + str(old) + ", new: " + str(new) + ")" )
        return old
    else:
        return new


def compute_distance(new_ticks, old_ticks):
    delta = new_ticks - old_ticks
    return float(delta) / ENCODER_TICKS_PER_REV * 2 * pi


def compute_velocity(new_ticks, old_ticks, new_time):
    global time
    return (compute_distance(new_ticks, old_ticks)) / (new_time - time).to_sec()


def set_initial_position(empty):
    global time, x, y, th

    position = None

    # Wait for parameter to become available
    while True:
        try:
            position = rospy.get_param("/reset/position")
        except KeyError:
            print("Initial position not yet available...")
            sleep(0.1)
        else:
            break

    time = rospy.Time.now()

    x = position['x']
    y = position['y']
    th = position['orientation']

    broadcast_transform(x, y, th, time)
    publish_odom(x, y, th, 0, 0, 0, time)



if __name__ == '__main__':
    try:
        rospy.init_node('encoder_to_odom')

        ENCODER_TICKS_PER_REV = rospy.get_param("/encoder/ticks_per_rev")
        WHEEL_RADIUS = rospy.get_param("/wheel/diameter") / 2
        WHEEL_SEPARATION_WIDTH = rospy.get_param("/wheel/separation/horizontal")
        WHEEL_SEPARATION_LENGTH = rospy.get_param("/wheel/separation/vertical")

        encoder_sub = rospy.Subscriber('encoder', Encoders, update)
        reset_sub = rospy.Subscriber('reset', Empty, set_initial_position)

        odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
        odom_broadcaster = tf.TransformBroadcaster()

        set_initial_position(None)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
