#!/usr/bin/env python

import rospy
from ac_msgs.msg import drive_params, drive_values
from std_msgs.msg import Bool, Float64


# function to map from one range to another, similar to arduino
def arduino_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min


class StateMaintainer:
    """
    This class maintains the state of the emergency flag. It is needed to ensure
    that the emergency flags are seen by the callback loop.
    """
    def __init__(self):
        self.eflag = False

def drive_callback(data, state):
    if(not state.eflag):
        velocity = data.velocity
        angle = data.angle
    else:
        velocity = 0.0
        angle = 0.5
        print("Emergency Stop Activated")
    print("Velocity: ", velocity, "Angle: ", angle)
    # Do the computation
    if velocity >= 0.5:
        print("Clamping velocity of %f to 0.5" % (velocity,))
        velocity = 0.5
    speed_msg = Float64(velocity)
    turn_msg = Float64(angle)
    speed_pub.publish(speed_msg)
    turn_pub.publish(turn_msg)


def eStop_callback(data, state):
    state.eflag=data

# Main function
if __name__ == '__main__':
    print("Emergency Stop Layer Initialized")
    rospy.init_node('emergency_stop_filter')
    state = StateMaintainer()
    speed_pub = rospy.Publisher('commands/motor/duty_cycle', Float64, queue_size=10)
    turn_pub = rospy.Publisher('commands/servo/position', Float64, queue_size=10)
    em_sub = rospy.Subscriber('eStop', Bool, eStop_callback, state)
    dr_param_sub = rospy.Subscriber("drive_parameters", drive_params, drive_callback, state)
    rospy.spin()
