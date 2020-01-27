#!/bin/sh

scp -r ./src/* nvidia@tegra-ubuntu.local:~/AutoCar/src
# scp -r ./slam/* car:/home/ubuntu/Documents/AutoCar/slam
# ssh car 'cd /home/ubuntu/Documents/AutoCar; catkin_make; source /home/ubuntu/Documents/AutoCar/devel/setup.bash'
