#!/usr/bin/env python

import rospy
from ac_msgs.msg import drive_params, drive_values


class CurrentState:
    def __init__(self):
        self.var1 = dict()
        self.var2 = dict()
        self.cdict = dict()

    def lineval(self):
        """
        Returns a line worth of info formatted like a CSV row, with a new line
        at the end.
        Exchanges var1 and 2
        """
        linestring = str()
        for key in self.vars:
            linestring = linestring + self.vars[key] + ","
        return (linestring + "\n")

class Value:
    def __init__(self):
        self._value = None
        self._pastvalue = None
    
    @property
    def value():
        return (self._pastvalue, self._value)

    @value.setter
    def value(value):
        self._pastvalue = value

def eStop_callback(state):
    """
    Update state with new value of eStop_callback
    """
    state.vars[eStop]

# Main function
if __name__ == '__main__':
    print("Data Logger Initialized")
    rospy.init_node('data_logger')
    state = CurrentState()
    em_sub = rospy.Subscriber('eStop', Bool, eStop_callback, state)
    dr_param_sub = rospy.Subscriber("drive_parameters", drive_params, drive_callback, state)
    rospy.spin()
