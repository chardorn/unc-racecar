#!/usr/bin/env python
import math
import time

import rospy
from sensor_msgs.msg import LaserScan
from ac_msgs.msg import drive_params
import numpy as np
from numpy import interp


def get_range(data, theta):
    # Input: 	data: LiDAR scan data
    # theta: The angle at which the distance is required
    # OUTPUT: distance of scan at angle theta
    index_float = interp(theta, [data.angle_min, data.angle_max], [0, len(data.ranges) - 1])
    index = int(index_float)
    print("range: " + str(data.ranges[index]))
    # Return the LiDAR scan value at that index
    # Do some error checking for NaN and ubsurd values
    ranges = data.ranges[index-2:index+3]
    r = np.mean(ranges)
    return r


def get_steering_angle(lowpoint, highpoint):
    """
    This function returns the steering angle with the appropriate signage
    X<-HighPoint
    |\
    | \
    |  \
    |   \
    |    \
    L-----X<-LowPoint
    :param lowpoint: The point closest in the y dimension to the car, as a tuple (x,y)
    :param highpoint: The point furthest from the car in the y dimension, as a tuple (x,y)
    :return: angle in radians
    """
    delx = float(highpoint[0])-float(lowpoint[0])
    dely = float(highpoint[1])-float(lowpoint[1])
    hyp = math.sqrt(delx**2 + dely**2)
    theta = -1*math.asin(delx/hyp)
    return theta


def get_coords(theta, range):
    """
    This maps polar to rectangular coordinates given the LiDAR's frame of reference.
    We rotate 90deg to right to reorient to polar origin
    :param theta: LiDAR theta
    :param range: LiDAR range
    :return:
    """
    y = math.cos(theta)*range
    x = -1 * math.sin(theta)*range
    return x, y


def laser_subscriber(msg, thetas):
    """
    I've hijacked this method to provide a visual of the thetas we're using
    """
    # publisher = rospy.Publisher("editscan", LaserScan, queue_size=10)
    # indices = interp(thetas, [msg.angle_min, msg.angle_max], [0, len(msg.ranges) - 1])
    # map(lambda x: int(x), indices)
    ## Drop everything but what we want.
    # for i in range(len(msg.ranges)):
    #    if i not in indices:
    #        msg.ranges[i] = 0
    #        msg.intensities = 0
    # publisher.publish(msg)
    pass


def main():
    print("Geometric Controller Initialized")
    rospy.init_node('geometric_controller_filter')
    pub = rospy.Publisher("drive_parameters", drive_params)
    while not rospy.core.is_shutdown():
        msg = rospy.client.wait_for_message('scan', LaserScan)
        print(msg.angle_min, msg.angle_max, msg.angle_increment)
        TOP = 1
        BOT = 0
        thetas = 3*math.pi/24
        thetar = [-math.pi / 2, (-math.pi / 2)+thetas]
        rangesr = [get_range(msg, t) for t in thetar]
        pointsr = [(rangesr[BOT], 0), get_coords(thetar[TOP], rangesr[TOP])]
        thetasr = get_steering_angle(pointsr[TOP], pointsr[BOT])
        print('pointsr', pointsr)
        print('thetasR', thetasr)

        thetal = [math.pi / 2, (math.pi / 2)-thetas]
        rangesl = [get_range(msg, t) for t in thetal]
        pointsl = [(-rangesl[BOT], 0), get_coords(thetal[TOP], rangesl[TOP])]
        thetasl = get_steering_angle(pointsl[TOP], pointsl[BOT])
        print('pointsl', pointsl)
        print('thetasL', thetasl)

        """
        Weight the thetas by how far the max distance is to the wall.
        This should keep us away from walls.
        """
        maxr = max(rangesr)**2
        maxl = max(rangesl)**2
        rweight = maxr/(maxr+maxl)
        lweight = maxl/(maxr+maxl)
        print("Weights:", lweight, "|", rweight)

        thetas = thetasl*lweight + thetasr*rweight
        steering_input = interp(thetas, [np.radians(-20), np.radians(20)], [-100, 100])
        print("ThetaR:", thetas, "|Steering Angle:", steering_input)
        message = drive_params()
        message.angle = steering_input
        message.velocity = 15
        pub.publish(message)


if __name__ == '__main__':
    main()
