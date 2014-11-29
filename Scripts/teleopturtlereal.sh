#! /bin/bash -x

#export ROS_HOSTNAME=192.168.43.88
#export ROS_MASTER_URI=http://192.168.43.124:11311
export ROS_HOSTNAME=192.168.25.13
export ROS_MASTER_URI=http://192.168.25.13:11311
source /opt/ros/hydro/setup.bash
roslaunch turtlebot_teleop keyboard_teleop.launch
