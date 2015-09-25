#!/usr/bin/env python

'''This runs the PbD system (i.e. the backend).'''

# Python imports
import sys

# Core ROS imports come first.
import rospy
import signal
from pr2_arm_control.arm_controls import ArmControls
from std_srvs.srv import Empty


if __name__ == '__main__':

    # Register as a ROS node.
    rospy.init_node('pr2_arm_control', anonymous=True)

    realtime = True
    if len(sys.argv) > 1:
        realtime = sys.argv[1]
        if realtime == 'False':
            realtime = False
        elif realtime == 'True':
            realtime = True
        else:
            realtime = True

    # Run the system
    client = rospy.ServiceProxy('clear_octomap', Empty)
    client.wait_for_service()
    client()
    arm_controls = ArmControls(realtime)
    while(not rospy.is_shutdown()):
        arm_controls.update()
        rospy.sleep(0.01)

