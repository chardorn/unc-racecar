# unc-racecar

Master repository for autonomous car.

**Instructions:**

To make all the nodes start running, terminal enter:
```
./start_car_tmux_windows.sh
```
This will open up a series of windows 0 through 5. You can switch between them to check to make sure everything works correctly by pressing Ctrl-B once and then the number of the window. 
In Window 0, you can run a control program. For example:
```
python ~/unc-racecar/src/charController/oscillator.py
```


**PREREQUISITES:**
Jetson TX2 or Jetson Nano
Running Linux 16.04 or 18.04
VESC already configured
Hokuyo Lidar or YDLidar


Follow these instructions to make a catkin workspace;
http://wiki.ros.org/catkin/Tutorials/create_a_workspace

Follow these instructions for setting ROS (Melodic)
http://wiki.ros.org/melodic/Installation/Ubuntu

To change between LIDAR scanners - between hokuyo and ydliidar, uncomment the line in start_car_tmux_windows.sh which specifies

Additional packages to be downloaded include:
ros-kinetic-ackermann-msgs
ros-kinetic-serial

and https://github.com/EAIBOT/ydlidar

In order to run ./start_car_tmux_windows, you must first install tmux, by calling apt install tmux
To close a tmux window, call:

```
tmux kill-session -t car-control
```

You may also have to go into the folder emergency stop filter and make the python files executavle by:
chmod +x estop_filter.py estop_sender.py
