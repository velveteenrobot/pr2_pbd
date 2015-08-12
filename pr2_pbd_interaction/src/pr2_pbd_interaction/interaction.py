'''Main interaction event handler. Receives speech and GUI commands and
sends events out to the system.'''

# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import roslib
roslib.load_manifest('pr2_pbd_interaction')
import rospy

# System builtins
from collections import Counter
import signal
import threading

# ROS builtins
from visualization_msgs.msg import MarkerArray

# Local
from pr2_arm_control.msg import ArmMode, Side, GripperState
from pr2_pbd_interaction.arms import Arms
from session import Session
from pr2_pbd_interaction.msg import (
    ArmState, ActionStep, ArmTarget, Object, GripperAction,
    ArmTrajectory, ExecutionStatus, GuiCommand)
from pr2_pbd_interaction.srv import Ping, PingResponse
from pr2_pbd_speech_recognition.msg import Command
from pr2_social_gaze.msg import GazeGoal
from response import Response
from robot_speech import RobotSpeech
from world import World

# ######################################################################
# Module level constants
# ######################################################################

EXECUTION_Z_OFFSET = -0.00
UPDATE_WAIT_SECONDS = 0.1
BASE_LINK = 'base_link'
# How fast to move between arm targets. NOTE(mbforbes): I'm unconvinced
# this is actually used anywhere. See arm.py: MOVE_TO_JOINTS_VELOCITY.
DEFAULT_VELOCITY = 0.2


# ######################################################################
# Classes
# ######################################################################

class Interaction:
    '''Interaction is the multiplexer of commands received into system
    actions.

    Interaction receives speech commands (recognized_command) as well as
    GUI commands (gui_command) and sends these off into the system to be
    processed. Interaction holds a small amount of state to support
    recording trajectories.

    This is the core class of the PbD "backend"; it can run on the robot
    or on the desktop.
    '''
    # TODO(mbforbes): Refactor trajectory busiens into new class.
    # TODO(mbforbes): Document class attributes in docstring.

    def __init__(self):
        # Create main components.
        self.world = World()
        self.arms = Arms(World.tf_listener)
        self.session = Session(object_list=self.world.get_frame_list())

        # ROS publishers and subscribers.
        self._viz_publisher = rospy.Publisher(
            'visualization_marker_array', MarkerArray)
        rospy.Subscriber(
            'recognized_command', Command, self._speech_command_cb)
        rospy.Subscriber('gui_command', GuiCommand, self._gui_command_cb)

        # Initialize trajectory recording state.
        self._is_recording_motion = False
        self._arm_trajectory = None
        self._trajectory_start_time = None

        # This is the main mechanism by which code is executed. A
        # Response as a combination of a function to call and a
        # parameter. Responses are created here to be triggered by
        # commands. Once a Response is respond(...)ed, a robot speech
        # utterance and a gaze action are created and then executed.
        # (This happens internally in Response.respond(...)).
        self.responses = {
            Command.TEST_MICROPHONE: Response(
                self._empty_response,
                [RobotSpeech.TEST_RESPONSE, GazeGoal.NOD]),
            Command.RELAX_RIGHT_ARM: Response(self._relax_arm, Side.RIGHT),
            Command.RELAX_LEFT_ARM: Response(self._relax_arm, Side.LEFT),
            Command.OPEN_RIGHT_HAND: Response(self._open_hand, Side.RIGHT),
            Command.OPEN_LEFT_HAND: Response(self._open_hand, Side.LEFT),
            Command.CLOSE_RIGHT_HAND: Response(self._close_hand, Side.RIGHT),
            Command.CLOSE_LEFT_HAND: Response(self._close_hand, Side.LEFT),
            Command.STOP_EXECUTION: Response(self._stop_execution, None),
            Command.DELETE_ALL_STEPS: Response(self._delete_all_steps, None),
            Command.DELETE_LAST_STEP: Response(self._delete_last_step, None),
            Command.FREEZE_RIGHT_ARM: Response(self._freeze_arm, Side.RIGHT),
            Command.FREEZE_LEFT_ARM: Response(self._freeze_arm, Side.LEFT),
            Command.CREATE_NEW_ACTION: Response(self._create_action, None),
            Command.EXECUTE_ACTION: Response(self._execute_action, None),
            Command.NEXT_ACTION: Response(self._next_action, None),
            Command.PREV_ACTION: Response(self._previous_action, None),
            Command.SAVE_POSE: Response(self._save_step, None),
            Command.RECORD_OBJECT_POSE: Response(
                self._record_object_pose, None),
            Command.START_RECORDING_MOTION: Response(
                self._start_recording, None),
            Command.STOP_RECORDING_MOTION: Response(self._stop_recording, None)
        }

        # Span off a thread to run the update loops.
        threading.Thread(
            group=None, target=self._update, name='interaction_update_thread'
        ).start()

        # Register signal handlers for program termination.
        # TODO(mbforbes): Test that these are really catching the
        # signals. I think we might have to pass disable_signals=True to
        # rospy.init_node(...), though I'm not sure.
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGQUIT, self._signal_handler)
        rospy.on_shutdown(self._on_shutdown)

        # The PbD backend is ready.
        rospy.loginfo('Interaction initialized.')
        self._ping_srv = rospy.Service(
            'interaction_ping', Ping, self._interaction_ping)

    # ##################################################################
    # Internal ("private" methods)
    # ##################################################################

    # The following methods are 'core' to the running of the program.

    def _signal_handler(self, signal, frame):
        '''Intercept quit signals (like ^C) in order to clean up (save
        experiment state) before exiting.'''
        rospy.loginfo('Interaction signal handler intercepted signal; saving.')
        self.session.save_current_action()
        # NOTE(mbforbes): We don't call sys.exit(0) here because that
        # would prevent cleanup from happening outside interaction
        # (e.g. in the node that's running it).

    def _interaction_ping(self, __):
        '''This is the service that is provided so that external nodes
        know the interaction (i.e. PbD) is ready.

        Args:
            __ (PingRequest): unused

        Returns:
            PingResponse: empty
        '''
        return PingResponse()

    def _on_shutdown(self):
        '''This is mostly for debugging: log that the interaction node
        itself is being shutdown.'''
        rospy.loginfo('Interaction node shutting down.')

    def update(self):
        '''General update for the main loop.

        This is called continuously from the node.
        This pauses for 100ms at the end of every
        run before returning.
        '''
        # Loop while not shutdown.
        while not rospy.is_shutdown():
            # Update arms.
            self.arms.update()
            if self.arms.status != ExecutionStatus.NOT_EXECUTING:
                if self.arms.status != ExecutionStatus.EXECUTING:
                    self._end_execution()

            # Record trajectory step.
            if self._is_recording_motion:
                self._save_arm_to_trajectory()

            # Update the current action if there is one.
            if self.session.n_actions() > 0:
                action = self.session.get_current_action()
                action.update_viz()

                # TODO(mbforbes): Do we ever have r/l target(s)? When does
                # this happen?
                for side in [Side.RIGHT, Side.LEFT]:
                    target = action.get_requested_target(side)
                    if target is not None:
                        self.arms.start_move_to_pose(target, side)
                        action.reset_targets(side)

                # Update any changes to steps that need to happen.
                action.delete_requested_steps()
                states = self._get_arm_states()
                action.change_requested_steps(
                    states[Side.RIGHT], states[Side.LEFT])

                # If the objects in the world have changed, update the
                # action with them.
                if self.world.update():
                    rospy.loginfo('The world has changed.')
                    self.session.get_current_action().update_objects(
                        self.world.get_frame_list())

            # This is the pause between update runs. Note that this doesn't
            # guarantee an update rate, only that there is this amount of
            # pause between udpates.
            rospy.sleep(UPDATE_WAIT_SECONDS)

        # At this point, we have shut down.
        self.session.save_current_action()
        rospy.loginfo("Interaction update thread exiting.")

    # The following methods receive commands from speech / GUI and
    # process them. These are the multiplexers.

    def _speech_command_cb(self, command):
        '''Callback for when a "speech" command is received.

        Note that Commands can actually be received from speech OR the
        GUI. They are called "speech commands" because they include all
        commands that can be issued with speech. The few commands that
        can be issued through the GUI and not through speech come as
        GUICommands, and are processed below in _gui_command_cb(...).

        Args:
            command (Command): The command received from speech OR the
                GUI.
        '''
        # We extract the command string as we use it a lot.
        strCmd = command.command
        if strCmd in self.responses.keys():
            rospy.loginfo(
                '\033[32m' + 'Calling response for command ' + strCmd +
                '\033[0m')
            response = self.responses[strCmd]

            if ((not self.arms.is_executing()) or strCmd ==
                    Command.STOP_EXECUTION):
                response.respond()
            else:
                rospy.logwarn(
                    'Ignoring speech command during execution: ' + strCmd)
        else:
            rospy.logwarn('This command (' + strCmd + ') is unknown.')

    def _gui_command_cb(self, command):
        '''Callback for when a GUICommand is received.

        Note that a GUICommand is not any command received from the GUI;
        it is specifically a command that is only possible from the GUI.
        Commands sent from the GUI that are also possible via speech are
        sent via Command messages and handled in the
        _speech_command_cb(...) function above.

        Args:
            command (GUICommand): The command received from the GUI.
        '''
        # We extract the command string as we use it a lot.
        strCmd = command.command

        # Because the GUI commands involve selecting actions or steps
        # within actions, we have two prerequisites: first, we cannot be
        # currently executing an action, and second, we must have at
        # least one action.
        if not self.arms.is_executing():
            if self.session.n_actions() > 0:
                if strCmd == GuiCommand.SWITCH_TO_ACTION:
                    # Command: switch to a specified action.
                    action_no = command.param
                    self.session.switch_to_action(
                        action_no,
                        self.world.get_frame_list())
                    response = Response(
                        self._empty_response,
                        [RobotSpeech.SWITCH_SKILL + str(action_no),
                            GazeGoal.NOD])
                    response.respond()
                elif strCmd == GuiCommand.SELECT_ACTION_STEP:
                    # Command: select a step in the current action.
                    step_no = command.param
                    self.session.select_action_step(step_no)
                    rospy.loginfo('Selected action step ' + str(step_no))
                else:
                    # Command: unknown. (Currently impossible.)
                    rospy.logwarn('This command (' + strCmd + ') is unknown.')
            else:
                # No actions; warn user.
                response = Response(
                    self._empty_response,
                    [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE])
                response.respond()
        else:
            # Currently executing; ignore command.
            rospy.logwarn(
                'Ignoring GUI command during execution: ' + strCmd)

    # The following methods are selected from commands (either GUI or
    # speech) and then called from within a Response objects's
    # respond(...) function. They follow the same pattern of their
    # accepted and returned values.

    def _open_hand(self, arm_index):
        '''Opens gripper on the indicated side.

        Args:
            arm_index (int): Side.RIGHT or Side.LEFT

        Returns:
            [str, int]: a speech response and a GazeGoal.* constant

        '''
        # First, open the hand if it's closed.
        if self.arms.set_gripper_state(arm_index, GripperState.OPEN):
            # Hand was closed, now open.
            speech_response = Response.open_responses[arm_index]
            if self.session.n_actions() > 0:
                # If we're currently programming, save that as a step.
                self._save_gripper_step(arm_index, GripperState.OPEN)
                speech_response = (
                    speech_response + ' ' + RobotSpeech.STEP_RECORDED)
            return [speech_response, Response.glance_actions[arm_index]]
        else:
            # Hand was already open; complain.
            return [
                Response.already_open_responses[arm_index],
                Response.glance_actions[arm_index]]

    def _close_hand(self, arm_index):
        '''Closes gripper on the indicated side.

        Args:
            arm_index (int): Side.RIGHT or Side.LEFT

        Returns:
            [str, int]: a speech response and a GazeGoal.* constant
        '''
        # First, close the hand if it's open.
        if Arms.set_gripper_state(arm_index, GripperState.CLOSED):
            # Hand was open, now closed.
            speech_response = Response.close_responses[arm_index]
            if self.session.n_actions() > 0:
                # If we're currently programming, save that as a step.
                self._save_gripper_step(arm_index, GripperState.CLOSED)
                speech_response = (
                    ' '.join([speech_response, RobotSpeech.STEP_RECORDED]))
            return [speech_response, Response.glance_actions[arm_index]]
        else:
            # Hand was already closed; complain.
            return [
                Response.already_closed_responses[arm_index],
                Response.glance_actions[arm_index]]

    def _relax_arm(self, arm_index):
        '''Relaxes / releases arm on the indicated side.

        Args:
            arm_index (int): Side.RIGHT or Side.LEFT

        Returns:
            [str, int]: a speech response and a GazeGoal.* constant
        '''
        # Release the arm. Response depens on whether it was previously
        # frozen or already released.
        if self.arms.set_arm_mode(arm_index, ArmMode.RELEASE):
            return [
                Response.release_responses[arm_index],
                Response.glance_actions[arm_index]]
        else:
            return [
                Response.already_released_responses[arm_index],
                Response.glance_actions[arm_index]]

    def _freeze_arm(self, arm_index):
        '''Freezes / holds / stiffens arm on the indicated side.

        Args:
            arm_index (int): Side.RIGHT or Side.LEFT

        Returns:
            [str, int]: a speech response and a GazeGoal.* constant
        '''
        # Freeze the arm. Response depens on whether it was previously
        # relaxed or already frozen.
        if self.arms.set_arm_mode(arm_index, ArmMode.HOLD):
            return [
                Response.hold_responses[arm_index],
                Response.glance_actions[arm_index]]
        else:
            return [
                Response.already_holding_responses[arm_index],
                Response.glance_actions[arm_index]]

    def _create_action(self, __=None):
        '''Creates a new empty action.

        Args:
            __ (Object): unused, default: None

        Returns:
            [str, int]: a speech response and a GazeGoal.* constant
        '''
        self.world.clear_all_objects()
        self.session.new_action()
        return [
            RobotSpeech.SKILL_CREATED + ' ' +
            str(self.session.current_action_index), GazeGoal.NOD]

    def _next_action(self, __=None):
        '''Switches to next action.

        Args:
            __ (Object): unused, default: None

        Returns:
            [str, int]: a speech response and a GazeGoal.* constant
        '''
        if self.session.n_actions() > 0:
            if self.session.next_action(self.world.get_frame_list()):
                return [
                    RobotSpeech.SWITCH_SKILL + ' ' +
                    str(self.session.current_action_index), GazeGoal.NOD]
            else:
                return [
                    RobotSpeech.ERROR_NEXT_SKILL + ' ' +
                    str(self.session.current_action_index), GazeGoal.SHAKE]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def _previous_action(self, __=None):
        '''Switches to previous action.

        Args:
            __ (Object): unused, default: None

        Returns:
            [str, int]: a speech response and a GazeGoal.* constant
        '''
        if self.session.n_actions() > 0:
            if self.session.previous_action(self.world.get_frame_list()):
                return [
                    RobotSpeech.SWITCH_SKILL + ' ' +
                    str(self.session.current_action_index),
                    GazeGoal.NOD]
            else:
                return [
                    RobotSpeech.ERROR_PREV_SKILL + ' ' +
                    str(self.session.current_action_index),
                    GazeGoal.SHAKE]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def _delete_last_step(self, __=None):
        '''Deletes last step of the current action.

        Args:
            __ (Object): unused, default: None

        Returns:
            [str, int]: a speech response and a GazeGoal.* constant
        '''
        if self.session.n_actions() > 0:
            if self.session.n_frames() > 0:
                self.session.delete_last_step()
                return [RobotSpeech.LAST_POSE_DELETED, GazeGoal.NOD]
            else:
                return [RobotSpeech.SKILL_EMPTY, GazeGoal.SHAKE]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def _delete_all_steps(self, __=None):
        '''Deletes all steps in the current action.

        Args:
            __ (Object): unused, default: None

        Returns:
            [str, int]: a speech response and a GazeGoal.* constant
        '''
        if self.session.n_actions() > 0:
            if self.session.n_frames() > 0:
                self.session.clear_current_action()
                return [RobotSpeech.SKILL_CLEARED, GazeGoal.NOD]
            else:
                return [RobotSpeech.SKILL_EMPTY, None]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def _stop_execution(self, __=None):
        '''Stops ongoing execution.

        Args:
            __ (Object): unused, default: None

        Returns:
            [str, int]: a speech response and a GazeGoal.* constant
        '''
        if self.arms.is_executing():
            self.arms.stop_execution()
            return [RobotSpeech.STOPPING_EXECUTION, GazeGoal.NOD]
        else:
            return [RobotSpeech.ERROR_NO_EXECUTION, GazeGoal.SHAKE]

    def _start_recording(self, __=None):
        '''Starts recording continuous motion.

        Args:
            __ (Object): unused, default: None

        Returns:
            [str, int]: a speech response and a GazeGoal.* constant
        '''
        if self.session.n_actions() > 0:
            if not self._is_recording_motion:
                self._is_recording_motion = True
                self._arm_trajectory = ArmTrajectory()
                self._trajectory_start_time = rospy.Time.now()
                return [RobotSpeech.STARTED_RECORDING_MOTION, GazeGoal.NOD]
            else:
                return [RobotSpeech.ALREADY_RECORDING_MOTION, GazeGoal.SHAKE]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def _stop_recording(self, __=None):
        '''Stops recording continuous motion.

        Args:
            __ (Object): unused, default: None

        Returns:
            [str, int]: a speech response and a GazeGoal.* constant
        '''
        if self._is_recording_motion:
            self._is_recording_motion = False
            traj_step = ActionStep()
            traj_step.type = ActionStep.ARM_TRAJECTORY

            waited_time = self._arm_trajectory.timing[0]
            for i in range(len(self._arm_trajectory.timing)):
                self._arm_trajectory.timing[i] -= waited_time
                self._arm_trajectory.timing[i] += rospy.Duration(0.1)

            self._fix_trajectory_ref()
            # Note that [:] is a shallow copy, copying references to
            # each element into a new list.
            traj_step.armTrajectory = ArmTrajectory(
                self._arm_trajectory.rArm[:],  # (ArmState[])
                self._arm_trajectory.lArm[:],  # (ArmState[])
                self._arm_trajectory.timing[:],  # (duration[])
                self._arm_trajectory.rRefFrame,  # (uint8)
                self._arm_trajectory.lRefFrame,  # (uint8)
                self._arm_trajectory.rRefFrameObject,  # (Object)
                self._arm_trajectory.lRefFrameObject  # (Object)
            )
            traj_step.gripperAction = GripperAction(
                GripperState(self.arms.get_gripper_state(Side.RIGHT)),
                GripperState(self.arms.get_gripper_state(Side.LEFT))
            )
            self.session.add_step_to_action(
                traj_step, self.world.get_frame_list())
            self._arm_trajectory = None
            self._trajectory_start_time = None
            return [RobotSpeech.STOPPED_RECORDING_MOTION + ' ' +
                    RobotSpeech.STEP_RECORDED, GazeGoal.NOD]
        else:
            return [RobotSpeech.MOTION_NOT_RECORDING, GazeGoal.SHAKE]

    def _save_step(self, __=None):
        '''Saves current arm state as an action step.

        Args:
            __ (Object): unused, default: None

        Returns:
            [str, int]: a speech response and a GazeGoal.* constant
        '''
        if self.session.n_actions() > 0:
            states = self._get_arm_states()
            step = ActionStep()
            step.type = ActionStep.ARM_TARGET
            step.armTarget = ArmTarget(
                states[Side.RIGHT],  # rArm (ArmState)
                states[Side.LEFT],  # lArm (ArmState)
                DEFAULT_VELOCITY,  # rArmVelocity (float64)
                DEFAULT_VELOCITY  # lArmVelocity (float64)
            )
            step.gripperAction = GripperAction(
                GripperState(self.arms.get_gripper_state(Side.RIGHT)),
                GripperState(self.arms.get_gripper_state(Side.LEFT))
            )
            self.session.add_step_to_action(step, self.world.get_frame_list())
            return [RobotSpeech.STEP_RECORDED, GazeGoal.NOD]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def _record_object_pose(self, __=None):
        '''Makes the robot look for a table and objects.

        Only does anything when at least one action has been created.

        Args:
            __ (Object): unused, default: None

        Returns:
            [str, int]: a speech response and a GazeGoal.* constant
        '''
        if self.world.update_object_pose():
            if self.session.n_actions() > 0:
                self.session.get_current_action().update_objects(
                    self.world.get_frame_list())
            return [RobotSpeech.START_STATE_RECORDED, GazeGoal.NOD]
        else:
            return [RobotSpeech.OBJECT_NOT_DETECTED, GazeGoal.SHAKE]

    def _empty_response(self, responses):
        '''Default response to speech commands; returns what it is
        passed.

        Args:
            [str, int]: a speech response and a GazeGoal.* constant

        Returns:
            [str, int]: a speech response and a GazeGoal.* constant
        '''
        return responses

    def _execute_action(self, __=None):
        '''Starts the execution of the current action.

        This saves the action before starting it.

        Args:
            __ (Object): unused, default: None

        Returns:
            [str, int]: a speech response and a GazeGoal.* constant
        '''
        # We must *have* a current action.
        if self.session.n_actions() > 0:
            # We must have also recorded steps (/poses/frames) in it.
            if self.session.n_frames() > 1:
                # Save curent action and retrieve it.
                self.session.save_current_action()
                action = self.session.get_current_action()

                # Now, see if we can execute.
                if action.is_object_required():
                    # We need an object; check if we have one.
                    if self.world.update_object_pose():
                        # An object is required, and we got one. Execute.
                        self.session.get_current_action().update_objects(
                            self.world.get_frame_list())
                        self.arms.start_execution(action, EXECUTION_Z_OFFSET)
                    else:
                        # An object is required, but we didn't get it.
                        return [
                            RobotSpeech.OBJECT_NOT_DETECTED, GazeGoal.SHAKE]
                else:
                    # No object is required: start execution now.
                    self.arms.start_execution(action, EXECUTION_Z_OFFSET)

                # Reply: starting execution.
                return [RobotSpeech.START_EXECUTION + ' ' +
                        str(self.session.current_action_index), None]
            else:
                # No steps / poses / frames recorded.
                return [RobotSpeech.EXECUTION_ERROR_NOPOSES + ' ' +
                        str(self.session.current_action_index), GazeGoal.SHAKE]
        else:
            # No actions.
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    # The following are "normal" private helper functions; they aren't
    # called from within a Response, and serve to help the above
    # functions.

    def _save_gripper_step(self, arm_index, gripper_state):
        '''Saves an action step that involves a gripper state change.

        Args:
            arm_index (int): Side.RIGHT or Side.LEFT
            gripper_state (int): GripperState.OPEN or
                GripperState.CLOSED
        '''
        if self.session.n_actions() > 0:
            states = self._get_arm_states()
            step = ActionStep()
            step.type = ActionStep.ARM_TARGET
            step.armTarget = ArmTarget(
                states[Side.RIGHT],  # rArm (ArmSTate)
                states[Side.LEFT],  # lArm (ArmState)
                DEFAULT_VELOCITY,  # rArmVelocity (float64)
                DEFAULT_VELOCITY  # lArmVelocity (float 64)
            )
            new_gripper_states = [
                self.arms.get_gripper_state(Side.RIGHT),
                self.arms.get_gripper_state(Side.LEFT)
            ]
            new_gripper_states[arm_index] = gripper_state
            step.gripperAction = GripperAction(
                GripperState(new_gripper_states[Side.RIGHT]),
                GripperState(new_gripper_states[Side.LEFT])
            )
            self.session.add_step_to_action(step, self.world.get_frame_list())

    def _fix_trajectory_ref(self):
        '''Makes the reference frame of continuous trajectories
        uniform.

        This means finding the dominant reference frame of the
        trajectory (for right and left arms separately), and then
        altering all steps in the trajctory to be relative to the same
        reference frame (again, separate for right and left arms).
        '''
        # First, get objects from the world.
        frame_list = self.world.get_frame_list()

        # Next, find the dominant reference frame (e.g. robot base,
        # an object).
        t = self._arm_trajectory
        r_ref_n, r_ref_obj = self._find_dominant_ref(t.rArm, frame_list)
        l_ref_n, l_ref_obj = self._find_dominant_ref(t.lArm, frame_list)

        # Next, alter all trajectory steps (ArmState's) so that they use
        # the dominant reference frame as their reference frame.
        for i in range(len(self._arm_trajectory.timing)):
            t.rArm[i] = World.convert_ref_frame(
                self._arm_trajectory.rArm[i],  # arm_frame (ArmState)
                r_ref_n,  # ref_frame (int)
                r_ref_obj  # ref_frame_obj (Objet)
            )
            t.lArm[i] = World.convert_ref_frame(
                self._arm_trajectory.lArm[i],  # arm_frame (ArmState)
                l_ref_n,  # ref_frame (int)
                l_ref_obj  # ref_frame_obj (Objet)
            )

        # Save the dominant ref. frame no./name in the trajectory for
        # reference.
        t.rRefFrame = r_ref_n
        t.lRefFrame = l_ref_n
        t.rRefFrameObject = r_ref_obj
        t.lRefFrameObject = l_ref_obj

    def _find_dominant_ref(self, arm_traj, frame_list):
        '''Finds the most dominant reference frame in a continuous
        trajectory.

        Args:
            arm_traj (ArmState[]): List of arm states that form the arm
                trajectory.
            frame_list ([Object]): List of Object (as defined by
                Object.msg), the current reference frames.

        Returns:
            (int, Object): Tuple of the dominant reference frame's
                number (as one of the constants available in ArmState to
                be set as ArmState.refFrame) and Object (as in
                Object.msg).
        '''
        # Cycle through all arm states and check their reference frames.
        # Whichever one is most frequent becomes the dominant one.
        robot_base = Object(name=BASE_LINK)
        ref_counts = Counter()
        for arm_state in arm_traj:
            # We only track objects that
            if arm_state.refFrame == ArmState.ROBOT_BASE:
                ref_counts[robot_base] += 1
            elif arm_state.refFrameObject in frame_list:
                ref_counts[arm_state.refFrameObject] += 1
            else:
                rospy.logwarn(
                    'Ignoring object with reference frame name '
                    + arm_state.refFrameObject.name
                    + ' because the world does not have this object.')

        # Get most common obj.
        dominant_ref_obj = ref_counts.most_common(1)[0][0]

        # Find the frame number (int) and return with the object.
        return World.get_ref_from_name(dominant_ref_obj.name), dominant_ref_obj

    def _save_arm_to_trajectory(self):
        '''Saves current arm state into continuous trajectory.'''
        if self._arm_trajectory is not None:
            states = self._get_arm_states()
            self._arm_trajectory.rArm.append(states[Side.RIGHT])
            self._arm_trajectory.lArm.append(states[Side.LEFT])
            self._arm_trajectory.timing.append(
                rospy.Time.now() - self._trajectory_start_time)

    def _get_arm_states(self):
        '''Returns the current arms states as a list of two ArmStates.

        Returns:
            [ArmState]: A list (of length two, one for each arm) of
                ArmState objects. Right first, then left.
        '''
        # TODO(mbforbes): Perhaps this entire method should go in
        # the Arms class?
        abs_ee_poses = [
            Arms.get_ee_state(Side.RIGHT),  # (Pose)
            Arms.get_ee_state(Side.LEFT)]  # (Pose)
        joint_poses = [
            Arms.get_joint_state(Side.RIGHT),  # ([float64])
            Arms.get_joint_state(Side.LEFT)]  # ([float64])

        states = [None, None]
        rel_ee_poses = [None, None]

        for arm_index in [Side.RIGHT, Side.LEFT]:
            nearest_obj = self.world.get_nearest_object(
                abs_ee_poses[arm_index])
            if not World.has_objects() or nearest_obj is None:
                # Arm state is absolute (relative to robot's base_link).
                states[arm_index] = ArmState(
                    ArmState.ROBOT_BASE,  # refFrame (uint8)
                    abs_ee_poses[arm_index],  # ee_pose (Pose)
                    joint_poses[arm_index],  # joint_pose ([float64])
                    Object()  # refFrameObject (Object)
                )
            else:
                # Arm state is relative (to some object in the world).
                rel_ee_poses[arm_index] = World.transform(
                    abs_ee_poses[arm_index],  # pose (Pose)
                    BASE_LINK,  # from_frame (str)
                    nearest_obj.name  # to_frame (str)
                )
                states[arm_index] = ArmState(
                    ArmState.OBJECT,  # refFrame (uint8)
                    rel_ee_poses[arm_index],  # ee_pose (Pose)
                    joint_poses[arm_index],  # joint_pose [float64]
                    nearest_obj  # refFrameObject (Object)
                )
        return states

    def _end_execution(self):
        '''Says a response and performs a gaze action for when an action
        execution ends.'''
        rospy.loginfo("Execution ended. Status: " + str(self.arms.status))
        if self.arms.status == ExecutionStatus.SUCCEEDED:
            # Execution completed successfully.
            Response.say(RobotSpeech.EXECUTION_ENDED)
            Response.perform_gaze_action(GazeGoal.NOD)
        elif self.arms.status == ExecutionStatus.PREEMPTED:
            # Execution stopped early (preempted).
            Response.say(RobotSpeech.EXECUTION_PREEMPTED)
            Response.perform_gaze_action(GazeGoal.SHAKE)
        else:
            # Couldn't solve for joint positions (IK).
            Response.say(RobotSpeech.EXECUTION_ERROR_NOIK)
            Response.perform_gaze_action(GazeGoal.SHAKE)
        # No matter what, we're not executing anymore.
        self.arms.status = ExecutionStatus.NOT_EXECUTING
