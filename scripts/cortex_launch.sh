#!/bin/sh

# Launch pigpio deamon
sudo pigpiod

# Export ROS env variables
export ROS_IP=$(hostname -I)
export ROS_MASTER_URI=http://$(hostname):11311

# Launch ROS
roslaunch robot_2018 cortex.launch raspberry:=true
