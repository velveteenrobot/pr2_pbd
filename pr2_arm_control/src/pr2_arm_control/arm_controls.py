#!/usr/bin/env python

'''This runs the PbD system (i.e. the backend).'''

# Core ROS imports come first.
import rospy
import signal
from pr2_arm_control.arm import Arm
from pr2_arm_control.msg import Side
from pr2_arm_control.arm_control_marker import ArmControlMarker
from tf import TransformListener

class ArmControls:
    '''Marker for visualizing the steps of an action.'''

    def __init__(self):
    	tf_listener = TransformListener()
    	r_arm = Arm(Side.RIGHT, tf_listener)
    	l_arm = Arm(Side.LEFT, tf_listener)
    	r_marker = ArmControlMarker(r_arm)
    	l_marker = ArmControlMarker(l_arm)
    	rospy.loginfo('Arm controls initialized.')