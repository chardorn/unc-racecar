#!/usr/bin/env python

import rospy
import numpy as np

from ac_msgs.msg import drive_params
from sensor_msgs.msg import LaserScan

from pure_pursuit import pure_pursuit
from calc_desired_torque import *
from calculate_goalpoint import *
from lidar_utils import *


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
    pub_goalpoint = rospy.Publisher('goal_point', PointStamped, queue_size=1)
    pub_foundgap = rospy.Publisher('gap_point', PointStamped, queue_size=1)
    rospy.on_shutdown(stop_car)
    torque_percentage = 0
    torque_calculator = TorqueCalculator()

    while not rospy.core.is_shutdown_requested():
        laser_data = rospy.client.wait_for_message('scan', LaserScan)
        prev_time = time.clock()  # calculate loop time

        goalpoint_at_infinity = calculate_goalpoint_manhattan(laser_data)

        scale = 1/math.sqrt(goalpoint_at_infinity[0]**2 + goalpoint_at_infinity[1]**2)
        goalpoint_in_rear_axel_coords = goalpoint_at_infinity*scale
        goalpoint_angle_rad = math.atan(goalpoint_at_infinity[1] / goalpoint_at_infinity[0])
        # Calculate the steering control output
        #   The coordinate system of Lidar and pure_pursuit is counterclockwise
        #   but the steering direction is clockwise, so we need to negate the output of pure_pursuit
        distance_to_goalpoint = 0.8
        steering_percentage = -pure_pursuit(goalpoint_angle_rad, distance_to_goalpoint) + 1 # Flip it
        print("Steering PCT: ", steering_percentage)
        # Calculate velocity
        # velocity = calculate_velocity(goalpoint_at_infinity)
        # print("Goalpoint-based velocity (m/s): ", velocity)

        # GP Based Calculate the torque control output
        #distance_to_goalpoint_at_infinity = goalpoint_at_infinity[0]**2+goalpoint_at_infinity[1]**2
        #if distance_to_goalpoint_at_infinity > 5:
        #    torque_percentage = 15000
        #else:
        #    torque_percentage = 4000

        # Constant speed
        #torque_percentage = 4000

        # Obstacle based
        front_distance_array = [get_distance_at_angle(laser_data, 10),
                                get_distance_at_angle(laser_data, 0),
                                get_distance_at_angle(laser_data, -10)]
        obstacle_in_front = False
        should_stop = False
        dist_threshold = 3
        stop_threshold = 0.3
        for d in front_distance_array:
            if d < dist_threshold:
                obstacle_in_front = True
            if d < stop_threshold:
                should_stop = True

        if should_stop:
            torque_percentage = 0.0
        elif obstacle_in_front:
            torque_percentage = 0.1 #5000
        else:
            torque_percentage = 0.2 #18000

        print(len(laser_data.ranges))

        # Steering Based speed
        #speed_steering = steering_percentage-.5
        #speed_steering = np.abs(speed_steering)
        #if speed_steering > .14:
        #    torque_percentage = 3000
        #else:
        #    torque_percentage = 10000

        # visualize the gap_point
        #gap_point = calculate_goalpoint_big_gap(laser_data)
        #gap_point = calculate_goalpoint_big_gap(laser_data)
        #gap_point_in_lidar_coords = rear_axle_to_lidar_coords(gap_point)
        #header = Header(stamp=rospy.Time.now(), frame_id="laser")
        #point = Point(gap_point_in_lidar_coords[0], gap_point_in_lidar_coords[1], 0)
        #gap_stamped = PointStamped(header=header, point=point)
        #pub_foundgap.publish(gap_stamped)

        # visualize the final goalpoint for purepursuit
        goalpoint_in_lidar_coords = rear_axle_to_lidar_coords(goalpoint_in_rear_axel_coords)
        p_x = goalpoint_in_lidar_coords[0]
        p_y = goalpoint_in_lidar_coords[1]
        header = Header(stamp=rospy.Time.now(), frame_id="laser")
        point = Point(p_x, p_y, 0)
        point_stamped = PointStamped(header=header, point=point)
        pub_goalpoint.publish(point_stamped)

        # send the torque and steering percentages to Teensy
        msg = drive_params()
        msg.angle = steering_percentage
        msg.velocity = torque_percentage
        pub_drive_param.publish(msg)

        # evaluate the execution time of the main control loop
        current_time = time.clock()
        delta_time_seconds = current_time - prev_time
        print("Main loop execution time in milliseconds: ", delta_time_seconds*1000)
        print('\n')

    pub_drive_param.publish(0, 0)
    pub_drive_param.unregister()

