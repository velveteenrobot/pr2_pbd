'''This runs the PbD backend (e.g. pbd_demo_robot).'''

# Core ROS imports come first.
import roslib
roslib.load_manifest('pr2_pbd_interaction')
import rospy

# Local
import interaction

if __name__ == '__main__':
    interaction.Interaction()
    # This is just so the Interaction instance doesn't get garbage
    # collected if this module were to finish.
    # TODO(mbforbes): Maybe should wait for intervals, checking if
    # shutdown? Or will this just return automatically when shutdown?
    rospy.spin()
