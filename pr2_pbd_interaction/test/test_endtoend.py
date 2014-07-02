#!/usr/bin/env python

'''End-to-end tests for PbD.'''

# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
PKG = 'pr2_pbd_interaction'
import roslib
roslib.load_manifest(PKG)
import rospy

# ROS builtins
from sensor_msgs.msg import JointState

# Local
from pr2_pbd_interaction.msg import Side, GuiCommand, GripperState
from pr2_pbd_speech_recognition.msg import Command
from pr2_pbd_interaction.srv import Ping
import unittest


# ######################################################################
# Constants
# ######################################################################

# How long to wait before any tests are run to let the system start up.
# Note that we do explicitly wait until the interaction node is
# initialized, but we can't easily do this for all of the nodes that
# it depends on.
PAUSE_STARTUP = 8.0

# How long to pause when starting a new test before publishing messages.
# If we start publishing messages immediately they are dropped by ROS or
# unseen for some reason.
PAUSE_SECONDS = 2.0

# How long to wait in-between querying the recorded joint states for its
# value. Note that the joints themselves publish updates at ~90Hz.
JOINT_REFRESH_PAUSE_SECONDS = 0.1

# Joints whose state we care about.
RELEVANT_JOINTS = [
    'l_gripper_joint',
    'r_gripper_joint'
]

# Joint settings---numbers that indicate joint status. Note that there
# is +/- 0.01 error in these.
GRIPPER_OPEN_POSITION = 0.08
GRIPPER_CLOSE_POSITION = 0.00
GRIPPER_EPSILON_POSITION = 0.01
GRIPPER_TOGGLE_TIME_SECONDS = 10.0


# ######################################################################
# Classes
# ######################################################################

class TestEndToEnd(unittest.TestCase):
    '''End-to-end tests for PbD.'''

    # ##################################################################
    # Core test infrastructure
    # ##################################################################

    def setUp(self):
        '''Ensures there is a valid interaction instance and wait until
        it's ready to rumble.'''
        # Ensure the interaction node is ready.
        rospy.wait_for_service('/interaction_ping')

        # Initialize some state.
        self.joint_positions = {}
        for joint in RELEVANT_JOINTS:
            self.joint_positions[joint] = None

        # Create our ROS message machinery.
        self.ping_srv = rospy.ServiceProxy('/interaction_ping', Ping)
        self.command_pub = rospy.Publisher('/recognized_command', Command)
        self.gui_command_pub = rospy.Publisher('/gui_command', GuiCommand)
        rospy.Subscriber('/gui_command', GuiCommand)
        rospy.Subscriber('/joint_states', JointState, self.joint_states_cb)

    # ##################################################################
    # Tests
    # ##################################################################

    def test_gripper_open_close(self):
        '''Tests that issuing 'speech' commands to open and close the
        gripper puts them in the desired state.'''
        self.check_alive()

        # Apparently need to wait a bit even here, or messages get
        # dropped.
        rospy.sleep(PAUSE_SECONDS)

        # Check opening/closing hands works
        self.command_pub.publish(Command(Command.OPEN_RIGHT_HAND))
        self.command_pub.publish(Command(Command.OPEN_LEFT_HAND))
        for joint in [side + '_gripper_joint' for side in ['r', 'l']]:
            self.assertJointCloseWithinTimeout(
                joint,
                GRIPPER_OPEN_POSITION,
                GRIPPER_EPSILON_POSITION,
                GRIPPER_TOGGLE_TIME_SECONDS
            )
        self.command_pub.publish(Command(Command.CLOSE_RIGHT_HAND))
        self.command_pub.publish(Command(Command.CLOSE_LEFT_HAND))
        for joint in [side + '_gripper_joint' for side in ['r', 'l']]:
            self.assertJointCloseWithinTimeout(
                joint,
                GRIPPER_CLOSE_POSITION,
                GRIPPER_EPSILON_POSITION,
                GRIPPER_TOGGLE_TIME_SECONDS
            )

        # Make sure nothing's crashed.
        self.check_alive()

    # ##################################################################
    # Helper methods
    # ##################################################################

    def check_alive(self):
        '''Ensures the interaction node is alive.'''
        self.ping_srv()
        self.assertTrue(True, "Interaction node should be alive.")

    def joint_states_cb(self, joint_state):
        '''Tracks relevant joint states.

        Args:
            joint_state (JointState): Sensor messages published by the
                PR2, reporting the state of its joints.
        '''
        names = joint_state.name
        positions = joint_state.position
        for joint in RELEVANT_JOINTS:
            if joint in names:
                self.joint_positions[joint] = positions[names.index(joint)]

    def are_floats_close(self, a, b, epsilon):
        '''Checks whether two floats are within epsilon of each
        other.

        Args:
            a (float): One number.
            b (float): The other number.
            epsilon (float): Acceptable wiggle room (+/-) between a and
                b.

        Returns:
            bool: Whether a and b are within epsilon of each other.
        '''
        # We try to do this in an overflow-friendly way, though it
        # probably isn't a big deal with our use cases and python.
        return a - epsilon <= b if a > b else b - epsilon <= a

    def assertJointCloseWithinTimeout(
            self, joint_name, expected_val, epsilon, timeout):
        '''Asserts that the position of the joint given by joint_name
        reaches "close to" expected_val within timeout seconds.

        (Note that this method is named in camelCase to match the other
        assert methods in the unittest library).

        Args:
            joint_name (str): Name of the joint to query.
            expected_val (float): Position joint must reach "close to".
            epsilon (float): Wiggle room (+/-) of joint reaching
                expected_val.
            timeout (float): How many seconds the joint has to reach
                close to the expected_val position.
        '''
        # We check this generously by checking once before and after the
        # timeout.
        if self.are_floats_close(
                self.joint_positions[joint_name], expected_val, epsilon):
            return

        # Check / sleep through timeout
        timeout_dur = rospy.Duration.from_sec(timeout)
        start = rospy.Time.now()
        while rospy.Time.now() - start < timeout_dur:
            if self.are_floats_close(
                    self.joint_positions[joint_name], expected_val, epsilon):
                return
            rospy.sleep(JOINT_REFRESH_PAUSE_SECONDS)

        # Check once after timeout.
        if self.are_floats_close(
                self.joint_positions[joint_name], expected_val, epsilon):
            return

        # Didn't make it; fail the test!
        self.assertFalse(
            True,
            "Joint %s never reached its expected value %f" %
            (joint_name, expected_val)
        )

# ######################################################################
# Program execution begins here
# ######################################################################

if __name__ == '__main__':
    rospy.init_node('test_endtoend')
    import rostest
    # Let the rest of the system get initialized
    rospy.loginfo("Waiting for system to start up.")
    rospy.sleep(PAUSE_STARTUP)
    rospy.loginfo("Done waiting for system to start up.")
    rostest.rosrun(
        PKG,  # package_name
        'test_end_to_end',  # test_name
        TestEndToEnd,  # test_case_class
    )
