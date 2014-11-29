#! /bin/bash -x

export ROS_HOSTNAME=192.168.25.13;
export ROS_MASTER_URI=http://192.168.25.13:11311
source ~/robotcom/setup.bash
rosrun teleopcom turtlebot_subscriber
