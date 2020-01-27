#!/bin/sh
#
# This script sets up and attaches to a tmux window running all of the car
# ROS scripts.
# Setup a work space called `work` with two windows

session="car-control"

tmux start-server

# create a new tmux session, starting vim from a saved session in the new window
tmux new-session -d -s $session

tmux new-window -t $session:1 -n "roscore"

tmux select-window -t $session:1
tmux send-keys "roscore" C-m
echo "Waiting for roscore to start"
sleep 3
tmux new-window -t $session:2 -n "VESC"
tmux select-window -t $session:2
tmux send-keys "roslaunch /home/nvidia/unc-racecar/src/vesc/vesc_driver/launch/vesc_driver_node.launch" C-m
echo "Waiting for VESC node to start"
sleep 2
tmux new-window -t $session:3 -n "estop"
tmux select-window -t $session:3
tmux send-keys "rosrun emergency_stop estop_filter.py" C-m
echo "Waiting for estop to start"
sleep 1
tmux new-window -t $session:4 -n "YDLIDAR"
tmux select-window -t $session:4
tmux send-keys "rosrun ydlidar ydlidar_node" C-m 
#tmux send-keys "rosrun urg_node urg_node _ip_address:=192.168.0.10" C-m
echo "Waiting for LIDAR node to start"
sleep 1

tmux new-window -t $session:5 -n "estop-sender"
tmux select-window -t $session:5
tmux send-keys "rosrun emergency_stop estop_sender.py" C-m
echo "Waiting for estop sender to start"
sleep 1

# Finally, attach to the tmux session.
tmux select-window -t $session:0
tmux attach-session -t $session

