#! /usr/bin/env python
import rospy
from ac_msgs.msg import drive_params
from std_msgs.msg import Bool
import time
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy
import threading

msg = drive_params()
msg.angle = 0.5
max_index = 540
mode = 0 #0 is manual, 1 is autonomous
stop_threads = False

def joy_callback(data):
    print("Call back!")
    global mode

    if data.buttons[1] == 1:
        setMode(1)
        #if not t1.isAlive():
        #    t1.start() #start autonomous()
    else:
        #if t1.isAlive():
        #    stop_threads = True
        #    t1.join()
        setMode(0)
        manual(data)

def manual(data):
    if data.axes[5] > 0:
        print("forward")
        if data.buttons[5] == 1.0:
            msg.velocity = 0.2
        else:
            msg.velocity = 0.1
    elif data.axes[5] < 0:
        print("backward")
        if data.buttons[5] == 1.0:
            msg.velocity = 0.2
        else:
            msg.velocity = -0.1
    else:
        msg.velocity = 0
    msg.angle = 0.5 - (data.axes[2] / 2.0)
    print(msg.angle)
    DriveParamPublisher.publish(msg)

def setMode(set):
    global mode
    mode = set

def getMode():
    global mode
    return mode

def autonomous(scans):
    print("..")
    mode = getMode()
    print("Mode: " + str(mode))
    if(mode == 1):
        print("Autonomous!")
        distances = scans.ranges
        num_scans = len(distances)
        if num_scans == 0:
            print("No scans!")
            return
        #start_index = int(num_scans * 0.25)
        #end_index = int(num_scans * 0.75)
        #index = start_index
        index = 0
        max_distance = 0.0
        max_index = num_scans / 2
        while(index < num_scans - 1):
            index = index + 1
            if distances[index] > max_distance:
                max_distance = distances[index]
                max_index = index
            else:
                continue
        print("Number of scans: " + str(num_scans))
        print("Max Index: " + str(max_index))
        #left is 0 and right is 1
        float_index = 0.1 + max_index - 0.1
        print("Float index: " + str(float_index))
        print("Num Scans: " + str(num_scans))
        servo_value = float_index / num_scans
        servo_value = 1.0 - servo_value
        print("Servo Value: " + str(servo_value))
        msg.angle = servo_value

        #Stop when a wall is directly in front
        if distances[num_scans / 2] < 0.5:
            msg.velocity = 0.0
        else:
            msg.velocity = 0.1
        DriveParamPublisher.publish(msg)

print("Setting up... ")
#t1 = threading.Thread(target=autonomous, args =(lambda : stop_threads, ))

DriveParamPublisher = rospy.Publisher("drive_parameters", drive_params, queue_size=10)
EStopPublisher = rospy.Publisher("eStop", Bool, queue_size=10)
#rospy.Subscriber('scan', LaserScan, lidar_callback)
rospy.Subscriber('scan', LaserScan, autonomous)
rospy.Subscriber('joy', Joy, joy_callback)
rospy.init_node("JoyControl")
rospy.spin()

#t1.join()
#t2.join()
