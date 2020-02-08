#! /usr/bin/env python
import rospy
from ac_msgs.msg import drive_params
from std_msgs.msg import Bool
import time
from race.msg import drive_param

rospy.init_node("BugControl")
DriveParamPublisher = rospy.Publisher("drive_parameters", drive_param, queue_size=10)
EStopPublisher = rospy.Publisher("eStop", Bool, queue_size=10)

msg = drive_param()
while(1):
    #EStopPublisher.publish(False)

    for a in range(100):
        time.sleep(0.01)
        turn =  (a / 100.0)
        msg.angle = turn
        msg.velocity = 1
        DriveParamPublisher.publish(msg)
    for b in range(100):
        time.sleep(0.01)
        turn =  1 - (b / 100.0)
        msg.angle = turn
        msg.velocity = 1
        DriveParamPublisher.publish(msg)
    continue
