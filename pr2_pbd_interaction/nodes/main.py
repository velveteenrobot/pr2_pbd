#!/usr/bin/env python

'''This runs the PbD system (i.e. the backend).'''

# Core ROS imports come first.
import rospy
import signal
import pr2_pbd_interaction
from pr2_pbd_interaction.interaction import Interaction

def signal_handler(signal, frame):
    # The following makes sure the state of a user study is saved, so that it can be recovered
    global interaction
    interaction.save_experiment_state()
    print 'Program Terminated!!'
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGQUIT, signal_handler)


if __name__ == '__main__':
    # Check whether we want code coverage, and start if so.
    global interaction

    # Run the system
    interaction_ = Interaction()
    rospy.spin()
    #while(not rospy.is_shutdown()):
    #    interaction.update()