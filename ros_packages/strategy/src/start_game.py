#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty


def end_of_game_cb(event):
    # Stop the game
    rospy.loginfo("End of game")
    rospy.loginfo(event)
    stop_pub = rospy.Publisher('stop', Empty, queue_size=1)
    stop_pub.publish(Empty())


def start(data):
    # start the timer of the game
    rospy.loginfo("Start of game")
    rospy.Timer(rospy.Duration(rospy.get_param("game/duration")), end_of_game_cb, oneshot=True)


def reset(data):
    rospy.loginfo("Reset of game")
    rospy.signal_shutdown("Reset the game!")

if __name__ == '__main__':
    try:
        rospy.init_node('start_game')
        rospy.Subscriber('start', Empty, start)
        rospy.Subscriber('reset', Empty, reset)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("A error occured.")
