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
GRIPPER_TOGGLE_TIME_SECONDS = 14.0


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
        # rospy.Subscriber('/gui_command', GuiCommand)
        rospy.Subscriber('/joint_states', JointState, self.joint_states_cb)

        # Apparently need to wait a bit even here, or messages get
        # dropped.
        rospy.sleep(PAUSE_SECONDS)

    # ##################################################################
    # Tests
    # ##################################################################

    def test_a_noaction_branches(self):
        '''This ideally exercises the "sorry, no action created yet"
        code that prevents requests from going through.

        The name "_a_" is in this test so that it runs first. This is
        because in PbD, once you create actions, you can never delete
        them, and it's not worth tearing down / bringing up PbD for
        each test case (it's also difficult to launch ROS nodes from
        within testing code).

        The one solace for this "test should run first" test, which is
        typically bad practice (and using "_a_" to make it run first,
        which is test runner-dependent), is that this test will still
        pass if other tests run first; it merely won't exercise the
        code that it's aiming to.
        '''
        # Ensure things are ready to go.
        self.check_alive()

        # Switch to nonexistant action
        self.gui_command_pub.publish(
            GuiCommand(GuiCommand.SWITCH_TO_ACTION, 50))

        # Switch to nonexistant step
        self.gui_command_pub.publish(
            GuiCommand(GuiCommand.SELECT_ACTION_STEP, 50))

        # Switch around actions
        self.command_pub.publish(Command(Command.NEXT_ACTION))
        self.command_pub.publish(Command(Command.PREV_ACTION))

        # Do naughty things within nonexistant action.
        self.command_pub.publish(Command(Command.DELETE_LAST_STEP))
        self.command_pub.publish(Command(Command.DELETE_ALL_STEPS))
        self.command_pub.publish(Command(Command.START_RECORDING_MOTION))
        self.command_pub.publish(Command(Command.SAVE_POSE))
        # No explicit check for record object pose, but it doesn't hurt
        self.command_pub.publish(Command(Command.RECORD_OBJECT_POSE))
        self.command_pub.publish(Command(Command.EXECUTE_ACTION))

        # Make sure nothing's crashed.
        self.check_alive()

    def test_gripper_open_close(self):
        '''Tests that issuing 'speech' commands to open and close the
        gripper puts them in the desired state.'''
        # Ensure things are ready to go.
        self.check_alive()

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

    def test_double_open_close(self):
        '''Tests that issuing a gripper open / close command twice
        doesn't break the robot and it stays in the desired state.
        '''
        # Ensure things are ready to go.
        self.check_alive()

        # Test settings (to avoid code duplication)
        positions = [GRIPPER_OPEN_POSITION, GRIPPER_CLOSE_POSITION]
        right_commands = [Command.OPEN_RIGHT_HAND, Command.CLOSE_RIGHT_HAND]
        left_commands = [Command.OPEN_LEFT_HAND, Command.CLOSE_LEFT_HAND]

        # Check each opening & closing, and do each twice.
        for state_idx in range(len(positions)):
            # First open/close once.
            self.command_pub.publish(Command(right_commands[state_idx]))
            self.command_pub.publish(Command(left_commands[state_idx]))
            for joint in [side + '_gripper_joint' for side in ['r', 'l']]:
                self.assertJointCloseWithinTimeout(
                    joint,
                    positions[state_idx],
                    GRIPPER_EPSILON_POSITION,
                    GRIPPER_TOGGLE_TIME_SECONDS
                )
            # Then, do the same thing again.
            self.command_pub.publish(Command(right_commands[state_idx]))
            self.command_pub.publish(Command(left_commands[state_idx]))
            for joint in [side + '_gripper_joint' for side in ['r', 'l']]:
                # Note that here we do "After" to ensure it doesn't just
                # pass immediately.
                self.assertJointCloseAfter(
                    joint,
                    positions[state_idx],
                    GRIPPER_EPSILON_POSITION,
                    GRIPPER_TOGGLE_TIME_SECONDS
                )

        # Make sure nothing's crashed.
        self.check_alive()

    def test_action_and_step_navigation(self):
        '''Tests creating / switching between actions, and creating /
        switching between steps.

        We could expose APIs within the system to query its state after
        issuing these commands, but at this point we're just going to
        exercise them and make sure things haven't crashed.
        '''
        # Ensure things are ready to go.
        self.check_alive()

        # Create / switch between actions.
        self.command_pub.publish(Command(Command.CREATE_NEW_ACTION))
        self.command_pub.publish(Command(Command.CREATE_NEW_ACTION))
        self.command_pub.publish(Command(Command.PREV_ACTION))
        self.command_pub.publish(Command(Command.NEXT_ACTION))

        # Try deleting last / all poses when there are none.
        self.command_pub.publish(Command(Command.DELETE_LAST_STEP))
        self.command_pub.publish(Command(Command.DELETE_ALL_STEPS))

        # Make some steps.
        self.command_pub.publish(Command(Command.SAVE_POSE))
        self.command_pub.publish(Command(Command.SAVE_POSE))
        self.command_pub.publish(Command(Command.SAVE_POSE))
        self.command_pub.publish(Command(Command.SAVE_POSE))

        # Switch between the steps.
        self.gui_command_pub.publish(
            GuiCommand(GuiCommand.SELECT_ACTION_STEP, 3))
        self.gui_command_pub.publish(
            GuiCommand(GuiCommand.SELECT_ACTION_STEP, 2))
        self.gui_command_pub.publish(
            GuiCommand(GuiCommand.SELECT_ACTION_STEP, 4))
        self.gui_command_pub.publish(
            GuiCommand(GuiCommand.SELECT_ACTION_STEP, 1))

        # Navigate away/to the action, switch to a step
        self.command_pub.publish(Command(Command.PREV_ACTION))
        self.command_pub.publish(Command(Command.NEXT_ACTION))
        self.gui_command_pub.publish(
            GuiCommand(GuiCommand.SELECT_ACTION_STEP, 2))

        # Delete a step and try to switch to it
        self.command_pub.publish(Command(Command.DELETE_LAST_STEP))
        self.gui_command_pub.publish(
            GuiCommand(GuiCommand.SELECT_ACTION_STEP, 4))

        # Delete all steps and try to switch to one
        self.command_pub.publish(Command(Command.DELETE_ALL_STEPS))
        self.gui_command_pub.publish(
            GuiCommand(GuiCommand.SELECT_ACTION_STEP, 2))

        # Final switching (depending on test run order, may switch to
        # different actions than we created here; hence, this comes last
        # in the test so we don't much with previously created steps.
        # Not that it matters... but nothing here should depend on the
        # existence or non-existence of steps).
        self.gui_command_pub.publish(
            GuiCommand(GuiCommand.SWITCH_TO_ACTION, 1))
        self.gui_command_pub.publish(
            GuiCommand(GuiCommand.SELECT_ACTION_STEP, 2))
        self.gui_command_pub.publish(
            GuiCommand(GuiCommand.SWITCH_TO_ACTION, 1))

        # Make sure nothing's crashed.
        self.check_alive()

    def test_simple_execution(self):
        '''Extremely simple execution test: just save poses in place and
        execute. Merely testing lack of system crash.'''
        # Ensure things are ready to go.
        self.check_alive()

        # Make an action and one pose.
        self.command_pub.publish(Command(Command.CREATE_NEW_ACTION))
        self.command_pub.publish(Command(Command.SAVE_POSE))

        # Executing here should say "no" because there aren't enough
        # poses.
        self.command_pub.publish(Command(Command.EXECUTE_ACTION))

        # Make an action and execute. It should work now.
        self.command_pub.publish(Command(Command.SAVE_POSE))
        self.command_pub.publish(Command(Command.EXECUTE_ACTION))

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

    def assertJointCloseAfter(
            self, joint_name, expected_val, epsilon, time):
        '''Asserts that the position of the joint given by joint_name
        is "close to" expected_val after time seconds.

        Note that this is different than "assertJointCloseWithinTimeout"
        as this method waits to assert until after time. This is useful
        if, for instance, you want to check something remains close to
        a value after a period of time.

        (Note that this method is named in camelCase to match the other
        assert methods in the unittest library).

        Args:
            joint_name (str): Name of the joint to query.
            expected_val (float): Position joint must reach "close to".
            epsilon (float): Wiggle room (+/-) of joint reaching
                expected_val.
            time (float): How many seconds to wait before asserting the
                joint is close to the expected_val position.
        '''
        rospy.sleep(time)
        self.assertTrue(
            self.are_floats_close(
                self.joint_positions[joint_name], expected_val, epsilon),
            "Joint %s isn't at its expected value %f" %
            (joint_name, expected_val)
        )

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
