#!/usr/bin/env python

import rospy
import numpy as np

from ac_msgs.msg import drive_params
from sensor_msgs.msg import LaserScan

from pure_pursuit import pure_pursuit
from calc_desired_torque import *
from calculate_goalpoint import *


def stop_car():
    pub = rospy.Publisher('drive_parameters', drive_params, queue_size=5)
    for i in range(50):
        msg = drive_params()
        msg.angle = 0.5
        msg.velocity = 0.0
        pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('geometric_controller', anonymous=True)
    pub_drive_param = rospy.Publisher('drive_parameters', drive_params)
    rospy.on_shutdown(stop_car)
    torque_percentage = 0.0

    while not rospy.core.is_shutdown_requested():
        time.sleep(0.01)
        prev_time = time.clock()  # calculate loop time

        # send the torque and steering percentages to Teensy
        msg = drive_params()
        msg.angle = 0.5
        msg.velocity = 0.3
        pub_drive_param.publish(msg)

        # evaluate the execution time of the main control loop
        current_time = time.clock()
        delta_time_seconds = current_time - prev_time
        print("Main loop execution time in milliseconds: ", delta_time_seconds*1000)
        print('\n')

    pub_drive_param.publish(0, 0)
    pub_drive_param.unregister()

