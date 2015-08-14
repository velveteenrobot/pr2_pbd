'''The in-program representation of a programmed action.'''

# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import roslib
roslib.load_manifest('pr2_pbd_interaction')
import rospy

# System builtins
import threading
import os

# ROS builtins
from geometry_msgs.msg import Vector3, Pose
import rosbag
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker

# Local
from action_step_marker import ActionStepMarker
from pr2_arm_control.msg import Side, GripperState
from pr2_pbd_interaction.msg import (
    ArmState, ActionStepSequence, ActionStep, ArmTarget,
    GripperAction, ArmTrajectory)


# ######################################################################
# Module level constants
# ######################################################################

# Filename related
FILENAME_BASE = 'Action'
DEFAULT_FILE_EXT = '.bag'

# Marker properties for little arrows drawn between consecutive steps.
LINK_MARKER_LIFETIME = rospy.Duration(2)
LINK_SCALE = Vector3(0.01, 0.03, 0.01)
LINK_COLOR = ColorRGBA(0.8, 0.8, 0.8, 0.3)  # sort of light gray

# ROS topics, etc.
TOPIC_MARKERS = 'visualization_marker_array'
TOPIC_BAG_SEQ = 'sequence'  # used when saving

# TODO(mbforbes): This should be in one module only.
BASE_LINK = 'base_link'


# ######################################################################
# Classes
# ######################################################################

class ProgrammedAction:
    '''Holds information for one action.'''

    _marker_publisher = None

    def __init__(self, action_index, step_click_cb):
        '''
        Args:
            action_index (int): The index of this action.
            step_click_cb (function(int)): The function to call when a
                step is clicked on (normally in the GUI). The function
                should take the UID of the step as calculated in
                ActionStepMarker.calc_uid(...).
        '''
        # Initialize a bunch of state.
        self.seq = ActionStepSequence()
        self.action_index = action_index
        self.step_click_cb = step_click_cb
        self.r_markers = []
        self.l_markers = []
        self.r_links = {}
        self.l_links = {}

        # NOTE(mbforbes): It appears that this is locking manipulation
        # of the internal sequence (self.seq). There have been race
        # conditions involving this (e.g. marker_click_cb(...)), in
        # which it was actually necessary to lock when accessing
        # self.r_markers and self.l_markers.
        #
        # In general, be aware the other code calling these methods
        # with with data about this class (like how many steps it holds)
        # is bad because that means the outside code is assuming that it
        # knows about state internal to this class, and that information
        # may not be true by the time the code here gets executed. This
        # is because there are several callbacks that trigger here so
        # we must reason asyncronously.
        #
        # Unless the information you have (e.g. about the number of
        # steps that exist) was learned while this lock was acquired,
        # you cannot assume it is true.
        self.lock = threading.Lock()

        if ProgrammedAction._marker_publisher is None:
            ProgrammedAction._marker_publisher = rospy.Publisher(
                TOPIC_MARKERS, MarkerArray)

    # ##################################################################
    # Static methods: Internal ("private")
    # ##################################################################

    @staticmethod
    def _update_if_edited(markers, arm_state):
        '''Updates the first marker in markers that is edited by setting
        its target to arm_state.

        Args:
            markers ([ActionStepMarker]): Call with self.r_markers or
                self.l_markers.
            arm_state (ArmState): New target for edited marker in
                markers.
        '''
        for marker in markers:
            if marker.is_edited:
                marker.set_target(arm_state)
                # This is structured to happen once, so we can return
                # now.
                return

    @staticmethod
    def _copy_action_step(action_step):
        '''Returns a copy of action_step.

        Args:
            action_step (ActionStep)

        Returns:
            ActionStep
        '''
        copy = ActionStep()
        copy.type = int(action_step.type)

        # Only the arm target is filled if this is an arm target, and
        # vice versa with a trajectory.
        if copy.type == ActionStep.ARM_TARGET:
            copy.armTarget = ArmTarget()
            copy.armTarget.rArmVelocity = float(
                action_step.armTarget.rArmVelocity)
            copy.armTarget.lArmVelocity = float(
                action_step.armTarget.lArmVelocity)
            copy.armTarget.rArm = ProgrammedAction._copy_arm_state(
                action_step.armTarget.rArm)
            copy.armTarget.lArm = ProgrammedAction._copy_arm_state(
                action_step.armTarget.lArm)
        elif copy.type == ActionStep.ARM_TRAJECTORY:
            copy.armTrajectory = ArmTrajectory()
            copy.armTrajectory.timing = action_step.armTrajectory.timing[:]
            for j in range(len(action_step.armTrajectory.timing)):
                copy.armTrajectory.rArm.append(
                    ProgrammedAction._copy_arm_state(
                        action_step.armTrajectory.rArm[j]))
                copy.armTrajectory.lArm.append(
                    ProgrammedAction._copy_arm_state(
                        action_step.armTrajectory.lArm[j]))
            copy.armTrajectory.rRefFrame = int(
                action_step.armTrajectory.rRefFrame)
            copy.armTrajectory.lRefFrame = int(
                action_step.armTrajectory.lRefFrame)
            # WARNING: the following is not really copying
            r_obj = action_step.armTrajectory.rRefFrameLandmark
            l_obj = action_step.armTrajectory.lRefFrameLandmark
            copy.armTrajectory.rRefFrameLandmark = r_obj
            copy.armTrajectory.lRefFrameLandmark = l_obj

        # NOTE(mbforbes): Conditions currently aren't copied (though
        # they also currently don't do anything, so I'm leaving for
        # now.)

        # Both arm targets and trajectories have a gripper action.
        copy.gripperAction = GripperAction(
            GripperState(action_step.gripperAction.rGripper.state),
            GripperState(action_step.gripperAction.lGripper.state)
        )
        return copy

    @staticmethod
    def _copy_arm_state(arm_state):
        '''Returns a copy of arm_state.

        Args:
            arm_state (ArmState)

        Returns:
            ArmState
        '''
        copy = ArmState()
        copy.refFrame = int(arm_state.refFrame)
        copy.joint_pose = arm_state.joint_pose[:]
        copy.ee_pose = Pose(
            arm_state.ee_pose.position, arm_state.ee_pose.orientation)
        # WARNING: the following is not really copying
        copy.refFrameLandmark = arm_state.refFrameLandmark
        return copy

    # ##################################################################
    # Instance methods: Public (API)
    # ##################################################################

    def get_name(self):
        '''Returns the name of the action.

        NOTE(mbforbes): There's no space between the string and the
        number because we use this as the base name when making the file
        name to save this action as (and it's easier to work without
        spaces when dealing with filenames).

        Returns:
            str: A name that is unique to this action index.

        '''
        return FILENAME_BASE + str(self.action_index)

    def add_action_step(self, step, object_list):
        '''Function to add a new step to the action.

        Args:
            step (ActionStep): The new step to add.
            object_list ([Landmark]): List of Landmark (as defined by
                Landmark.msg), the current reference frames.
        '''
        self.lock.acquire()
        self.seq.seq.append(self._copy_action_step(step))
        # We currently support arm targets and arm trajectories.
        # NOTE(mbforbes): It's unclear to me this is the best way to
        # support future step types in the system. Doesn't this just
        # mark one more spot in the code that needs to be changed to
        # implement another action type?
        if (step.type == ActionStep.ARM_TARGET
                or step.type == ActionStep.ARM_TRAJECTORY):
            # Create and append new action step markers.
            # NOTE(mbforbes): One of many instances of code duplication
            # b/c of right/left...
            last_step = self.seq.seq[-1]
            r_marker = ActionStepMarker(
                self.n_frames(),
                Side.RIGHT,
                last_step,
                self.marker_click_cb
            )
            r_marker.update_ref_frames(object_list)
            l_marker = ActionStepMarker(
                self.n_frames(),
                Side.LEFT,
                last_step,
                self.marker_click_cb
            )
            l_marker.update_ref_frames(object_list)
            self.r_markers.append(r_marker)
            self.l_markers.append(l_marker)

            # If we have any steps in this action, we link the previous
            # one to this new one.
            if self.n_frames() > 1:
                self.r_links[self.n_frames() - 1] = self._get_link(
                    Side.RIGHT, self.n_frames() - 1)
                self.l_links[self.n_frames() - 1] = self._get_link(
                    Side.LEFT, self.n_frames() - 1)
        self.lock.release()

    def update_objects(self, object_list):
        '''Updates the object list for all of this action's steps.

        Args:
            object_list ([Landmark]): List of Landmark (as defined by
                Landmark.msg), the current reference frames.
        '''
        self.lock.acquire()
        self._update_markers()
        for marker in self.r_markers + self.l_markers:
            marker.update_ref_frames(object_list)
        self.lock.release()

    def reset_targets(self, arm_index):
        '''Resets requests after reaching a previous target.

        Args:
            arm_index (int): Side.RIGHT or Side.LEFT
        '''
        self.lock.acquire()
        markers = self.r_markers if arm_index == Side.RIGHT else self.l_markers
        for marker in markers:
            marker.pose_reached()
        self.lock.release()

    def delete_requested_steps(self):
        '''Deletes steps whose deletions were were requested from
        interactive marker menus.

        Note that this function will only delete at most one step per
        call.
        '''
        self.lock.acquire()
        to_delete = None
        for i in range(len(self.r_markers)):
            if (self.r_markers[i].is_deleted or
                    self.l_markers[i].is_deleted):
                # We found something to delete. Mark and break (as we
                # delete only one thing).
                rospy.loginfo('Will delete step ' + str(i + 1))
                self.r_markers[i].is_deleted = False
                self.l_markers[i].is_deleted = False
                to_delete = i
                break

        # If we found anything to delete, delete it.
        if to_delete is not None:
            self._delete_step(to_delete)

        self.lock.release()

        # Update if we deleted anything.
        if to_delete is not None:
            self.update_viz()
            self._update_markers()

    def change_requested_steps(self, r_arm, l_arm):
        '''Change an arm step to the current end effector poses if
        requested through the interactive marker menu.

        Args:
            r_arm (ArmState)
            l_arm (ArmState)
        '''
        self.lock.acquire()
        ProgrammedAction._update_if_edited(self.r_markers, r_arm)
        ProgrammedAction._update_if_edited(self.l_markers, l_arm)
        self.lock.release()

    def get_requested_target(self, arm_index):
        '''Gets an arm step that might have been requested from the
        interactive marker menus for the arm_index arm.

        Args:
            arm_index (int): Side.RIGHT or Side.LEFT

        Returns:
            ArmState|None: The first arm step that was requested, or
                None if no arm steps were requested.

        '''
        # TODO(mbforbes): When does this happen? Ever?
        arm_state = None
        self.lock.acquire()
        markers = self.r_markers if arm_index == Side.RIGHT else self.l_markers
        for marker in markers:
            if marker.is_requested:
                arm_state = marker.get_target()
                # We only return one, so we can stop as soon as we find
                # one.
                # TODO(mbforbes): This used to return the last one, but
                # now it returns the first. It seemed like a bug that
                # it was returning the last, but it may have been a bug
                # with needed side-effects, so we should test this isn't
                # true.
                break
        self.lock.release()
        return arm_state

    def n_frames(self):
        '''Returns the number of steps in this action.

        Returns:
            int
        '''
        return len(self.seq.seq)

    def save(self, data_dir):
        '''Saves the action into a file.

        Args:
            data_dir (str): Directory in which to save the action file.
        '''
        if self.n_frames() > 0:
            self.lock.acquire()
            demo_bag = rosbag.Bag(data_dir + self._get_filename(), 'w')
            demo_bag.write(TOPIC_BAG_SEQ, self.seq)
            demo_bag.close()
            self.lock.release()
        else:
            rospy.logwarn(
                'Could not save action because it does not have any steps.')

    def load(self, data_dir):
        '''Loads an action from a file.

        Args:
            data_dir (str): Directory where this action's file should be
                located.
        '''
        filename = data_dir + self._get_filename()
        if os.path.exists(filename):
            self.lock.acquire()
            demo_bag = rosbag.Bag(filename)
            for dummy, msg, bag_time in demo_bag.read_messages(
                    topics=[TOPIC_BAG_SEQ]):
                rospy.loginfo(
                    'Reading demo bag file at time ' + str(bag_time.to_sec()))
                self.seq = msg
            demo_bag.close()
            self.lock.release()
        else:
            rospy.logwarn(
                'File does not exist, cannot load action: ' + filename)

    def reset_viz(self):
        '''Removes all visualization from Rviz.'''
        self.lock.acquire()

        # Destroy the action step markers.
        for marker in self.r_markers + self.l_markers:
            marker.destroy()

        # Mark the links for destruction.
        for i in self.r_links.keys():
            self.r_links[i].action = Marker.DELETE
            self.l_links[i].action = Marker.DELETE

        # Publish the link destructions.
        m_array = MarkerArray()
        for i in self.r_links.keys():
            m_array.markers.append(self.r_links[i])
        for i in self.l_links.keys():
            m_array.markers.append(self.l_links[i])
        self._marker_publisher.publish(m_array)

        # Reset internal data structures.
        self.r_markers = []
        self.l_markers = []
        self.r_links = {}
        self.l_links = {}
        self.lock.release()

    def marker_click_cb(self, uid, is_selected):
        '''Callback for when one of the markers is clicked.

        Goes over all other markers and un-selects them.

        Args:
            uid (int): The unique ID for the action step marker, as can
                be calculated by ActionStepMarker.calc_uid(...).
            is_selected(bool): Whether the marker denoted by uid was
                selected (True) or de-selected (False).

        '''
        self.lock.acquire()
        for marker in self.r_markers + self.l_markers:
            # If we match the one we've clicked on, select it.
            if marker.get_uid() == uid:
                marker.is_control_visible = is_selected
                marker.update_viz()
            else:
                # Otherwise, deselect it.
                if marker.is_control_visible:
                    marker.is_control_visible = False
                    marker.update_viz()

        # If we selected it, really click on it.
        if is_selected:
            self.step_click_cb(uid)
        self.lock.release()

    def select_step(self, step_id):
        '''Makes the interactive marker for the indicated action step
        selected by showing the 6D controls.

        Args:
            step_id (int): The unique ID for the action step marker, as
                can be calculated by ActionStepMarker.calc_uid(...).
        '''
        self.marker_click_cb(step_id, True)

    def initialize_viz(self, object_list):
        '''Initialize visualization.

        Args:
            object_list ([Landmark]): List of Landmark (as defined by
                Landmark.msg), the current reference frames.
        '''
        self.lock.acquire()
        for i in range(len(self.seq.seq)):
            step = self.seq.seq[i]
            # NOTE(mbforbes): It's unclear to me this is the best way to
            # support future step types in the system. Doesn't this just
            # mark one more spot in the code that needs to be changed to
            # implement another action type?
            if (step.type == ActionStep.ARM_TARGET or
                    step.type == ActionStep.ARM_TRAJECTORY):
                # Construct the markers.
                r_marker = ActionStepMarker(
                    i + 1,  # step_number
                    Side.RIGHT,  # arm_index
                    step,  # action_step
                    self.marker_click_cb  # marker_click_cb
                )
                l_marker = ActionStepMarker(
                    i + 1,  # step_number
                    Side.LEFT,  # arm_index
                    step,  # action_step
                    self.marker_click_cb  # marker_click_cb
                )

                # Update and add.
                r_marker.update_ref_frames(object_list)
                l_marker.update_ref_frames(object_list)
                self.r_markers.append(r_marker)
                self.l_markers.append(l_marker)

                # If we're not adding the first step, we should link the
                # last one to it.
                if i > 0:
                    self.r_links[i] = self._get_link(Side.RIGHT, i)
                    self.l_links[i] = self._get_link(Side.LEFT, i)

        self._update_markers()
        self.lock.release()

    def get_last_step(self):
        '''Returns the last step of the action.

        Returns:
            ActionStep
        '''
        self.lock.acquire()
        last_step = self.seq.seq[-1]
        self.lock.release()
        return last_step

    def delete_last_step(self):
        '''Deletes the last step of the action.'''
        self.lock.acquire()
        self._delete_step(len(self.seq.seq) - 1)
        self.lock.release()

    def is_object_required(self):
        '''Returns whether this action has any steps that are relative
        to objects in the world (instead of absolute).

        Returns:
            bool
        '''
        is_required = False
        self.lock.acquire()
        for step in self.seq.seq:
            # Extract reference frames.
            tp = step.type
            if tp == ActionStep.ARM_TARGET:
                r_ref = step.armTarget.rArm.refFrame
                l_ref = step.armTarget.lArm.refFrame
            else:
                # tp == ActionStep.ARM_TRAJECTORY
                r_ref = step.armTrajectory.rRefFrame
                l_ref = step.armTrajectory.lRefFrame

            # Logic: If either reference frame is to an object, we
            # return true.
            if r_ref == ArmState.OBJECT or l_ref == ArmState.OBJECT:
                is_required = True
                break
        self.lock.release()
        return is_required

    def get_gripper_states(self, arm_index):
        '''Returns the gripper states for all action steps for arm
        arm_index.

        Args:
            arm_index (int): Side.RIGHT or Side.LEFT

        Returns:
            [int]: Each element is either GripperState.OPEN or
                GripperState.CLOSED.
        '''
        self.lock.acquire()
        gripper_states = []
        for action_step in self.seq.seq:
            gact = action_step.gripperAction
            gs = gact.rGripper if arm_index == Side.RIGHT else gact.lGripper
            gripper_states += [gs.state]
        self.lock.release()
        return gripper_states

    def get_ref_frame_names(self, arm_index):
        '''Returns the names of the reference frame objects for all
        action steps for arm arm_index.

        Args:
            arm_index (int): Side.RIGHT or Side.LEFT

        Returns:
            [str]
        '''
        self.lock.acquire()
        ref_frame_names = []
        for action_step in self.seq.seq:
            target = action_step.armTarget
            arm = target.rArm if arm_index == Side.RIGHT else target.lArm
            ref_frame_names += [arm.refFrameLandmark.name]
        self.lock.release()
        return ref_frame_names

    def get_step(self, index):
        '''Returns a step of the action.

        Args:
            index (int): Index (0-based) of step to return.

        Returns:
            ActionStep|None: Returns None if no such step exists.
        '''
        # NOTE(mbforbes): For this lock to be meaningful, we have to
        # check that the index is valid within it.
        self.lock.acquire()
        n_steps = len(self.seq.seq)
        if index < 0 or index >= n_steps:
            rospy.logerr(
                "Requested step index " + str(index) + ", but only have " +
                str(n_steps) + " steps.")
            requested_step = None
        else:
            requested_step = self.seq.seq[index]
        self.lock.release()
        return requested_step

    def copy(self):
        '''Returns a copy of this instance.

        Returns:
            ProgrammedAction
        '''
        action = ProgrammedAction(self.action_index, self.step_click_cb)
        action.seq = ActionStepSequence()
        for action_step in self.seq.seq:
            copy = ProgrammedAction._copy_action_step(action_step)
            action.seq.seq.append(copy)
        return action

    def update_viz(self):
        '''Updates the visualization of the action.'''
        self.lock.acquire()
        self._update_links()
        m_array = MarkerArray()
        for i in self.r_links.keys():
            m_array.markers.append(self.r_links[i])
        for i in self.l_links.keys():
            m_array.markers.append(self.l_links[i])
        self._marker_publisher.publish(m_array)
        self.lock.release()

    def clear(self):
        '''Clears the action.'''
        self.reset_viz()
        self.lock.acquire()
        self.seq = ActionStepSequence()
        self.r_markers = []
        self.l_markers = []
        self.r_links = dict()
        self.l_links = dict()
        self.lock.release()

    # ##################################################################
    # Instance methods: Internal ("private")
    # ##################################################################

    def _get_link(self, arm_index, to_index):
        '''Returns a marker representing a link b/w two consecutive
        action steps (both must already exist).

        Args:
            arm_index (int): Side.RIGHT or Side.LEFT
            to_index (int): The index of the 'end' marker (the latter of
                the two).

        Returns:
            Marker
        '''
        markers = self.r_markers if arm_index == Side.RIGHT else self.l_markers
        start = markers[to_index - 1].get_absolute_position(is_start=True)
        end = markers[to_index].get_absolute_position(is_start=False)
        return Marker(
            type=Marker.ARROW,
            id=ActionStepMarker.calc_uid(arm_index, to_index),
            lifetime=LINK_MARKER_LIFETIME,
            scale=LINK_SCALE,
            header=Header(frame_id=BASE_LINK),
            color=LINK_COLOR,
            points=[start, end]
        )

    def _update_markers(self):
        '''Updates the markers after a change.'''
        for marker in self.r_markers + self.l_markers:
            marker.update_viz()

    def _delete_step(self, to_delete):
        '''(Actually) deletes a step from the action.

        NOTE(mbforbes): The lock should be acquired before calling this
        method.

        Args:
            to_delete (int): The index of the step to delete.
        '''
        rospy.loginfo('Deleting step: ' + str(to_delete))
        if len(self.r_links) > 0:
            self.r_links[self.r_links.keys()[-1]].action = Marker.DELETE
            self.l_links[self.l_links.keys()[-1]].action = Marker.DELETE
            self.r_links.pop(self.r_links.keys()[-1])
            self.l_links.pop(self.l_links.keys()[-1])

        self.r_markers[-1].destroy()
        self.l_markers[-1].destroy()
        for i in range(to_delete + 1, self.n_frames()):
            self.r_markers[i].decrease_id()
            self.l_markers[i].decrease_id()
        self.r_markers.pop(to_delete)
        self.l_markers.pop(to_delete)
        self.seq.seq.pop(to_delete)

    def _update_links(self):
        '''Updates the visualized links b/w action steps.'''
        for i in self.r_links.keys():
            self.r_links[i] = self._get_link(Side.RIGHT, i)
        for i in self.l_links.keys():
            self.l_links[i] = self._get_link(Side.LEFT, i)

    def _get_filename(self, ext=DEFAULT_FILE_EXT):
        '''Returns the filename for the bag that holds this action.

        Returns:
            str
        '''
        if ext[0] != '.':
            ext = '.' + ext
        return self.get_name() + ext
