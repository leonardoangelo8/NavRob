#! /bin/bash -x

export ROS_HOSTNAME=192.168.25.13;
export ROS_MASTER_URI=http://192.168.25.13:11311
source ~/robotcom/setup.bash
roslaunch turtlebot_teleop turtlebot_teleop_cpp.launch
