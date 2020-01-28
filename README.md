# unc-racecar

Master repository for autonomous car.

To change between LIDAR scanners - between hokuyo and ydliidar, uncomment in start_car_tmux_windows.sh

Additional packages to be downloaded include:
ros-kinetic-ackermann-msgs
ros-kinetic-serial

and https://github.com/EAIBOT/ydlidar

In order to run ./start_car_tmux_windows, you must first install tmux, by calling apt install tmux
To close a tmux window, call:
tmux kill-session -t car-control

You may also have to go into the folder emergency stop filter and make the python files executavle by:
chmod +x estop_filter.py estop_sender.py


