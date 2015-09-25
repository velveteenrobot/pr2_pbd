#!/usr/bin/env python

'''This runs the PbD system (i.e. the backend).'''

# Core ROS imports come first.
import rospy
import signal
from pr2_arm_control.arm_controls import ArmControls
from std_msgs.srv import Empty


if __name__ == '__main__':

    # Register as a ROS node.
    rospy.init_node('pr2_arm_control', anonymous=True)

    # Run the system
    arm_controls = ArmControls()
    client = rospy.ServiceProxy('clear_octomap', Empty)
    client()
    while(not rospy.is_shutdown()):
        arm_controls.update()
        rospy.sleep(0.5)
