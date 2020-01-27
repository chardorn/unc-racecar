# Getting started with AutoCar

## Table of Contents

1. [Requirements](#Requirements)
2. [Installation](#Installation)
3. [Execution](#Execution)

## Requirements

- ROS Indigo
- Ubuntu 14.04
- F1Tenth Platform
- Jetson TK1
- Hokuyo LiDAR UST-10LX

## Installation

Install ROS on the Jetson and on your development machine following the tutorials available at [F1Tenth](http://f1tenth.org)

## Execution

### What nodes to run

Make sure your terminals are sourced. (`source /home/nvidia/Autocar/devel/setup.bash`)
I appended the above command in the .bashrc file.

1. `roscore`

2. Communications with vesc

    `roslaunch /home/nvidia/AutoCar/src/vesc/vesc_driver/launch/vesc_driver_node.launch`

3. Emergency stop node (all torque (or velocity?) and steering commands are sent to this node. Then this node sends the command to the vesc driver node.)

    `rosrun emergency_stop estop_filter.py`

4. IMU Node (optional, and not installed yet):

    `rosrun razor_imu_9dof imu_node.py`

5. Run the node for lidar.

    `rosrun urg_node urg_node _ip_address:=192.168.0.10`

6. Controller

    `rosrun pure_pursuit manhattan_control.py`

## Network config

- Jetson's ethernet port ip address is 192.168.0.15

- Lidar's ip address is 192.168.0.10
