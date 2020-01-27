#!/usr/bin/env python
import time
import rospy
from ac_msgs.msg import drive_params
from std_msgs.msg import Bool


DriveParamPublisher = rospy.Publisher("drive_parameters", drive_params, queue_size=10)
rospy.init_node("data_collection_script")
msg = drive_params()

time.sleep(1)
msg.velocity = 50
msg.angle = 0
DriveParamPublisher.publish(msg)
time.sleep(.500)
#msg.velocity = -10
#msg.angle = 0
#DriveParamPublisher.publish(msg)
#time.sleep(.500)
msg.velocity = 0
msg.angle = 0
DriveParamPublisher.publish(msg)
