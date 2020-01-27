import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

rospy.init_node('lidar_reader', anonymous=True)
np.set_printoptions(precision=2)
while not rospy.core.is_shutdown_requested():
    laser_data = rospy.client.wait_for_message('scan', LaserScan)
    distances = laser_data.ranges
    center_index = len(distances) / 2
    mid_10_distances = distances[center_index - 5 : center_index + 5]
    print np.array(mid_10_distances)
