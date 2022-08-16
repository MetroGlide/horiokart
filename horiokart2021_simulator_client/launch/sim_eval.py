#!/usr/bin/env python

import rospy
import sys

def get_bool_param():
    param_name = '/hj_simulation'
    value = rospy.get_param(param_name)
    sys.stdout.write(str(value))
    return value