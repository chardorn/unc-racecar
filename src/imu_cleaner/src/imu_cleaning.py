#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

import math


class VariableMaintainer:
    def __init__(self):
        self._time = rospy.Time.now()
        self.lin_x = None
        self.lin_y = None
        self.lin_z = None
        self.est = [0, 0, -9.8]
        self.prev_err_est = [.1, .1, .1]

    @property
    def time(self):
        return self._time

    @time.setter
    def time(self, value):
        self._time = value


def publish_clean_imu(msg, vmaint):
    pub = rospy.Publisher('clean_imu', Imu)
    # Implement Kalman Filter
    # Error in measurement comes from covariance matrix. Only variances are provided by this IMU(Razor9DOF)
    kg = [vmaint.prev_err_est[0]/(vmaint.prev_err_est[0]+math.sqrt(msg.linear_acceleration_covariance[0])),
          vmaint.prev_err_est[1]/(vmaint.prev_err_est[1]+math.sqrt(msg.linear_acceleration_covariance[4])),
          vmaint.prev_err_est[2]/(vmaint.prev_err_est[2]+math.sqrt(msg.linear_acceleration_covariance[8]))]
    print("Kalman Gain is: ", kg)
    curr_est = [vmaint.est[0] + (kg[0] * (msg.linear_acceleration.x - vmaint.est[0])),
                vmaint.est[1] + (kg[1] * (msg.linear_acceleration.y - vmaint.est[1])),
                vmaint.est[2] + (kg[2] * (msg.linear_acceleration.z - vmaint.est[2]))]
    print("Current Estimate is: ", curr_est)
    error_est = [(1 - kg[0]) * vmaint.prev_err_est[0],
                 (1 - kg[1]) * vmaint.prev_err_est[1],
                 (1 - kg[2]) * vmaint.prev_err_est[2]]
    print("Estimate Error is: ", error_est)
    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = curr_est
    vmaint.est = curr_est
    vmaint.prev_err_est = error_est
    pub.publish(msg)


def main():
    # Collect first 50 entries and average them.
    rospy.init_node("imu_cleaner")
    vmaint = VariableMaintainer()
    c_val = rospy.client.wait_for_message('imu', Imu)
    vmaint.lin_x, vmaint.lin_y, vmaint.lin_z = c_val.linear_acceleration.x, c_val.linear_acceleration.y, c_val.linear_acceleration.z
    rospy.Subscriber('imu', Imu, publish_clean_imu, vmaint)
    rospy.spin()


if __name__ == '__main__':
    main()
