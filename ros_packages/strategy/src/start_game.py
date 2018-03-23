#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty


def end_of_game_cb(event):
    # Stop the game
    rospy.loginfo("End of game")
    stop_pub.publish(Empty())


def start(data):
    # start the timer of the game
    rospy.loginfo("Start of game")
    rospy.Timer(rospy.Duration(rospy.get_param("/game/duration")), end_of_game_cb, oneshot=True)


def reset(data):
    rospy.loginfo("Reset of game")
    rospy.shutdown()

if __name__ == '__main__':
    try:
        rospy.init_node('game')
        rospy.Subscriber('start', Empty, start)
        rospy.Subscriber('reset', Empty, reset)
        stop_pub = rospy.Publisher('stop', Empty, queue_size=1)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("A error occured.")
