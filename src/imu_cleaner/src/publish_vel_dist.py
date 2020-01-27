#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import Imu
from std_msgs.msg import *

import math


class VariableMaintainer:
    def __init__(self):
        self.time = False
        self.velocity = [0, 0, 0]
        self.position = [0, 0, 0]


def publish_velocity_distance(msg, vmaint):
    vel_pub = rospy.Publisher('calc_velocity', Vector3)
    pos_pub = rospy.Publisher('calc_position', Vector3)
    if not vmaint.time:  # This means this is the first run
        vmaint.time = msg.header.stamp.secs + msg.header.stamp.nsecs * math.pow(10, -9)
        vmaint.velocity = [0, 0, 0]
        vmaint.position = [0, 0, 0]
    else:
        c_time = msg.header.stamp.secs + msg.header.stamp.nsecs * math.pow(10, -9)
        tdelt = c_time - vmaint.time
        accel = [msg.linear_acceleration.x,
                 msg.linear_acceleration.y,
                 msg.linear_acceleration.z]
        map(lambda x: x * tdelt, accel)
        vmaint.velocity = [vmaint.velocity[i] + accel[i] for i in range(3)]
        map(lambda x: x * tdelt, accel)
        vmaint.position = [vmaint.velocity[i] + accel[i] for i in range(3)]
        vmaint.time = c_time
        v = Vector3(vmaint.velocity[0], vmaint.velocity[1], vmaint.velocity[2])
        p = Vector3(vmaint.position[0], vmaint.position[1], vmaint.position[2])
        vel_pub.publish(v)
        pos_pub.publish(p)


def main():
    # Collect first 50 entries and average them.
    rospy.init_node("publish_vel_dist")
    vmaint = VariableMaintainer()
    rospy.Subscriber('clean_imu', Imu, publish_velocity_distance, vmaint)
    rospy.spin()


if __name__ == '__main__':
    main()
