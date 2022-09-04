#!/bin/bash
echo ROS_IP=192.168.0.10 >> $HOME/.bashrc
echo ROS_MASTER_URI=http://$ROS_IP:11311 >> $HOME/.bashrc
echo "source /opt/ros/melodic/setup.bash" >> $HOME/.bashrc
echo "source $HOME/catkin_ws/devel/setup.bash" >> $HOME/.bashrc

source $HOME/catkin_ws/devel/setup.bash

#export ROS_IP=192.168.0.10
#export ROS_MASTER_URI=http://$ROS_IP:11311

roscore