#!/bin/bash
source ~/.bashrc

workspace=$(pwd)

sleep 1

gnome-terminal -t "can" -x bash -c "source ~/.bashrc;source /opt/ros/noetic/setup.bash; sudo -S sudo ip link set up can0 type can bitrate 1000000;candump can0;exec bash;"

sleep 1

gnome-terminal -t "launcher" -x bash -c "source ~/.bashrc;\
cd ${workspace};\
source devel/setup.bash;roslaunch arm_control arx5.launch;exec bash;"




