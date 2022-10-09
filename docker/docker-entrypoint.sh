#!/bin/bash
echo ROS_IP=192.168.0.10 >> $HOME/.bashrc
echo ROS_MASTER_URI=http://$ROS_IP:11311 >> $HOME/.bashrc
echo "source /opt/ros/noetic/setup.bash" >> $HOME/.bashrc
echo "source $HOME/catkin_ws/devel/setup.bash" >> $HOME/.bashrc

export ROS_IP=192.168.0.10
export ROS_MASTER_URI=http://$ROS_IP:11311

source /opt/ros/noetic/setup.bash
cd $HOME/catkin_ws
rosdep install -i -y --from-paths src/
catkin build
source $HOME/catkin_ws/devel/setup.bash

roscore