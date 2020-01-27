#!/usr/bin/env python
# This script probably won't end up being used--it still doesn't guarantee
# a straight-line path that avoids obstacles, and it's too slow. However, we'll
# preserve it for reference.

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

def index_to_angle(i):
    """ Takes an index into a LIDAR scan array and returns the associated
    angle, in degrees. """
    return -135.0 + (i / 1081.0) * 0.25

def get_normal(scan_data, index, neighbor_offset):
    laser_data = scan_data.ranges
    left_index = index + neighbor_offset
    right_index = index - neighbor_offset
    left_distance = 1e10
    right_distance = 1e10
    if left_index < len(laser_data):
        left_distance = laser_data[left_index]
    if right_index >= 0:
        right_distance = laser_data[right_index]
    center_distance = laser_data[index]
    left_difference = abs(left_distance - center_distance)
    right_difference = abs(right_distance - center_distance)
    target_index = right_index
    rotation_angle = -90.0
    if left_difference < right_difference:
        target_index = left_index
        rotation_angle = 90.0
    point_1 = get_point_at_index(scan_data, index)
    point_2 = get_point_at_index(scan_data, target_index)
    relative = point_2 - point_1
    relative /= np.linalg.norm(relative)
    if rotation_angle == 90.0:
        relative[0], relative[1] = -relative[1], relative[0]
    else:
        relative[0], relative[1] = relative[1], -relative[0]
    return relative

def get_bloated_point(scan_data, index, car_width = 0.175):
    measured_point = get_point_at_index(scan_data, index)
    normal = get_normal(scan_data, index, 4)
    return measured_point + normal * car_width

def get_bloated_polar_coordinate(scan_data, index):
    bloated_point = get_bloated_point(scan_data, index)
    distance = np.linalg.norm(bloated_point)
    normalized = bloated_point / distance
    degrees = (np.arcsin(normalized[1]) / math.pi) * 180.0
    return distance, degrees

def get_all_bloated_points(scan_data, sample_every = 10):
    result_count = len(scan_data.ranges) / sample_every
    if (len(scan_data.ranges) % sample_every) != 0:
        result_count += 1
    results = np.zeros((result_count, 2))
    cur_index = 0
    for i in range(len(scan_data.ranges)):
        if (i % sample_every) != 0:
            continue
        results[cur_index] = get_bloated_polar_coordinate(scan_data,i)
        cur_index += 1
    #return np.sort(results, 1)
    return results

def get_target_angle(bloated_data):
    current_max_distance = -1e10
    current_angle = 0.0
    for i in range(len(bloated_data)):
        v = bloated_data[i]
        if v[0] > current_max_distance:
            current_max_distance = v[0]
            current_angle = v[1]
    return current_angle

if __name__ == '__main__':
    rospy.init_node('obstacle_bloating', anonymous=True)
    rospy.on_shutdown(stop_car)

    while not rospy.core.is_shutdown_requested():
        laser_data = rospy.client.wait_for_message('scan', LaserScan)
        prev_time = time.clock()  # calculate loop time
        print "Got " + str(len(laser_data.ranges)) + " laser points"
        start = time.time()
        bloated_data = get_all_bloated_points(laser_data)
        target_angle = get_target_angle(bloated_data)
        duration = time.time() - start
        for i in range(len(bloated_data)):
            if (i % 10) != 0:
                continue
            print bloated_data[i]
        print "The angle with the largest distance is %f degrees." % (target_angle,)
        print "Doing that took %f ms" % (duration * 1000.0,)
        time.sleep(3)

