'''Represents a single 3D marker (in RViz) for visualizing the steps of
an action.'''

# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import rospy
import numpy
from geometry_msgs.msg import Quaternion, Vector3, Point, Pose
from std_msgs.msg import Header, ColorRGBA
import tf
from interactive_markers.interactive_marker_server import (
    InteractiveMarkerServer)
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import (
    Marker, InteractiveMarker, InteractiveMarkerControl,
    InteractiveMarkerFeedback)
import threading
from pr2_arm_control.msg import Side, GripperState

# ######################################################################
# Module level constants
# ######################################################################

DEFAULT_OFFSET = 0.09

# Marker options
# --------------
# Colors
COLOR_MESH_REACHABLE = ColorRGBA(0.0, 0.5, 1.0, 0.6)
COLOR_MESH_UNREACHABLE = ColorRGBA(0.5, 0.5, 0.5, 0.6)

# Gripper mesh related
ANGLE_GRIPPER_OPEN = 28 * numpy.pi / 180.0
ANGLE_GRIPPER_CLOSED = 0.0
STR_MESH_GRIPPER_FOLDER = 'package://pr2_description/meshes/gripper_v0/'
STR_GRIPPER_PALM_FILE = STR_MESH_GRIPPER_FOLDER + 'gripper_palm.dae'
STR_GRIPPER_FINGER_FILE = STR_MESH_GRIPPER_FOLDER + 'l_finger.dae'
STR_GRIPPER_FINGERTIP_FILE = STR_MESH_GRIPPER_FOLDER + 'l_finger_tip.dae'

# Other
INT_MARKER_SCALE = 0.2

# We might want to refactor this even further, as it's used throughout
# the code.
REF_FRAME = 'base_link'

# Other options
# --------------
ARM_NAMES = ['right', 'left']


# ######################################################################
# Classes
# ######################################################################

class ArmControlMarker:
    '''Marker for visualizing the steps of an action.'''

    _im_server = None
    _offset = DEFAULT_OFFSET

    def __init__(self, arm):
        '''
        Args:
            arm_index (int): Side.RIGHT or Side.LEFT
        '''
        if ArmControlMarker._im_server is None:
            im_server = InteractiveMarkerServer('interactive_arm_control')
            ArmControlMarker._im_server = im_server

        self._arm = arm
        self.is_requested = False
        self.is_control_visible = False
        self.is_edited = False

        self._sub_entries = None
        self._menu_handler = None
        self._prev_is_reachable = None
        self._pose = self._arm.get_ee_state()
        self._update_menu()
        self.update_viz()

    # ##################################################################
    # Static methods: Internal ("private")
    # ##################################################################

    @staticmethod
    def get_pose_from_transform(transform):
        '''Returns pose for transformation matrix.

        Args:
            transform (Matrix3x3): (I think this is the correct type.
                See ActionStepMarker as a reference for how to use.)

        Returns:
            Pose
        '''
        pos = transform[:3, 3].copy()
        rot = tf.transformations.quaternion_from_matrix(transform)
        return Pose(
            Point(pos[0], pos[1], pos[2]),
            Quaternion(rot[0], rot[1], rot[2], rot[3])
        )

    @staticmethod
    def get_matrix_from_pose(pose):
        '''Returns the transformation matrix for given pose.

        Args:
            pose (Pose)

        Returns:
            Matrix3x3: (I think this is the correct type. See
                ActionStepMarker as a reference for how to use.)
        '''
        pp, po = pose.position, pose.orientation
        rotation = [po.x, po.y, po.z, po.w]
        transformation = tf.transformations.quaternion_matrix(rotation)
        position = [pp.x, pp.y, pp.z]
        transformation[:3, 3] = position
        return transformation

    @staticmethod
    def _offset_pose(pose, constant=1):
        '''Offsets the world pose for visualization.

        Args:
            pose (Pose): The pose to offset.
            constant (int, optional): How much to scale the set offset
                by (scales ArmControlMarker._offset). Defaults to 1.

        Returns:
            Pose: The offset pose.
        '''
        transform = ArmControlMarker.get_matrix_from_pose(pose)
        offset_array = [constant * ArmControlMarker._offset, 0, 0]
        offset_transform = tf.transformations.translation_matrix(offset_array)
        hand_transform = tf.transformations.concatenate_matrices(
            transform, offset_transform)
        return ArmControlMarker.get_pose_from_transform(hand_transform)

    # ##################################################################
    # Instance methods: Public (API)
    # ##################################################################

    def get_uid(self):
        '''Returns a unique id for this marker.

        Returns:
            int: A number that is unique given the arm
                index.
        '''
        return self._arm.arm_index

    def destroy(self):
        '''Removes marker from the world.'''
        ArmControlMarker._im_server.erase(self._get_name())
        ArmControlMarker._im_server.applyChanges()

    def set_new_pose(self, new_pose, is_offset=False):
        '''Changes the pose of the action step to new_pose.

        Args:
            new_pose (Pose)
        '''
        if is_offset:
            self._pose = new_pose
        else:
            self._pose = ArmControlMarker._offset_pose(new_pose, -1)
        self.update_viz()

    def get_pose(self):
        '''Returns the pose of the action step.

        Returns:
            Pose
        '''
        return ArmControlMarker._offset_pose(self._pose)

    def update_viz(self):
        '''Updates visualization fully.'''
        self._update_viz_core()
        self._menu_handler.reApply(ArmControlMarker._im_server)
        ArmControlMarker._im_server.applyChanges()

    def pose_reached(self):
        '''Update when a requested pose is reached.'''
        self.is_requested = False

    def marker_feedback_cb(self, feedback):
        '''Callback for when an event occurs on the marker.

        Args:
            feedback (InteractiveMarkerFeedback)
        '''
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.set_new_pose(feedback.pose)
            self.update_viz()
        elif feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            # Set the visibility of the 6DOF controller.
            # This happens a ton, and doesn't need to be logged like
            # normal events (e.g. clicking on most marker controls
            # fires here).
            rospy.logdebug('Changing visibility of the pose controls.')
            self.is_control_visible = not self.is_control_visible
        else:
            # This happens a ton, and doesn't need to be logged like
            # normal events (e.g. clicking on most marker controls
            # fires here).
            rospy.logdebug('Unknown event: ' + str(feedback.event_type))


    def open_gripper_cb(self, __):
        self._arm.open_gripper()
        self._update_menu()

    def close_gripper_cb(self, __):
        self._arm.close_gripper()
        self._update_menu()

    def move_to_cb(self, __):
        '''Callback for when moving to a pose is requested.

        Args:
            __ (???): Unused
        '''

        target_joints = self._arm.get_ik_for_ee(
                self._pose, self._arm.get_joint_state())

        if target_joints is not None:
            time_to_pose = self._arm.get_time_to_pose(self.get_pose())

            thread = threading.Thread(
                group=None,
                target=self._arm.move_to_joints,
                args=(target_joints, time_to_pose),
                name='move_to_arm_state_thread'
            )
            thread.start()

            # Log
            side_str = self._arm.side()
            rospy.loginfo('Started thread to move ' + side_str + ' arm.')
        else:
            rospy.loginfo('Will not move ' + side_str + ' arm, because unreachable.')


    #### TODO... if necessary..
    def move_pose_to_cb(self, __):
        '''Callback for when a pose change to current is requested.

        Args:
            __ (???): Unused

        '''
        self.set_new_pose(self._arm.get_ee_state(), is_offset=True)

    # ##################################################################
    # Instance methods: Internal ("private")
    # ##################################################################

    def _is_reachable(self):
        '''Checks and returns whether there is an IK solution for this
        action step.

        Returns:
            bool: Whether this action step is reachable.
        '''

        target_joints = self._arm.get_ik_for_ee(
                self._pose, self._arm.get_joint_state())


        is_reachable = (target_joints is not None)

        # A bit more complicated logging to avoid spamming the logs
        # while still giving useful info. It now logs when reachability
        # is first calculated, or changes.
        report = False

        # See if we've set reachability for the first time.
        if self._prev_is_reachable is None:
            report = True
            reachable_str = (
                'is reachable' if is_reachable else 'is not reachable')
        # See if we've got a different reachability than we had before.
        elif self._prev_is_reachable != is_reachable:
            report = True
            reachable_str = (
                'is now reachable' if is_reachable else
                'is no longer reachable')

        # Log if it's changed.
        if report:
            rospy.loginfo(self._arm.side() + ':' + reachable_str)

        # Cache and return.
        self._prev_is_reachable = is_reachable
        return is_reachable

    def _get_name(self):
        '''Generates the unique name for the marker.

        Returns:
            str: A human-readable unique name for the marker.
        '''
        return 'arm' + str(self._arm.arm_index)

    def _update_menu(self):
        '''Recreates the menu when something has changed.'''
        self._menu_handler = MenuHandler()

        # Inset main menu entries.
        self._menu_handler.insert(
            'Move gripper here', callback=self.move_to_cb)
        self._menu_handler.insert(
            'Move marker to current gripper pose', callback=self.move_pose_to_cb)

        if self._is_hand_open():
            self._menu_handler.insert(
                'Close gripper',
                callback=self.close_gripper_cb)
        else:
            self._menu_handler.insert(
                'Open gripper',
                callback=self.open_gripper_cb)


        # Update.
        self._update_viz_core()
        self._menu_handler.apply(ArmControlMarker._im_server, self._get_name())
        ArmControlMarker._im_server.applyChanges()

    def _is_hand_open(self):
        '''Returns whether the gripper is open for this action step.

        Returns:
            bool
        '''
        return self._arm.get_gripper_state() == GripperState.OPEN

    def _update_viz_core(self):
        '''Updates visualization after a change.'''
        # Create a new IM control.
        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.BUTTON
        menu_control.always_visible = True
        frame_id = REF_FRAME
        pose = self.get_pose()

        # Multiplex marker types added based on action step type.
        # Handle "normal" steps (saved poses).
        menu_control = self._make_gripper_marker(
            menu_control, self._is_hand_open())

        # Make and add interactive marker.
        int_marker = InteractiveMarker()
        int_marker.name = self._get_name()
        int_marker.header.frame_id = frame_id
        int_marker.pose = pose
        int_marker.scale = INT_MARKER_SCALE
        self._add_6dof_marker(int_marker, True)
        int_marker.controls.append(menu_control)
        ArmControlMarker._im_server.insert(
            int_marker, self.marker_feedback_cb)

    def _add_6dof_marker(self, int_marker, is_fixed):
        '''Adds a 6 DoF control marker to the interactive marker.

        Args:
            int_marker (InteractiveMarker)
            is_fixed (bool): Looks like whether position is fixed (?).
                Currently only passed as True.
        '''
        # Each entry in options is (name, orientation, is_move)
        options = [
            ('rotate_x', Quaternion(1, 0, 0, 1), False),
            ('move_x', Quaternion(1, 0, 0, 1), True),
            ('rotate_z', Quaternion(0, 1, 0, 1), False),
            ('move_z', Quaternion(0, 1, 0, 1), True),
            ('rotate_y', Quaternion(0, 0, 1, 1), False),
            ('move_y', Quaternion(0, 0, 1, 1), True),
        ]
        for opt in options:
            name, orient, is_move = opt
            control = self._make_6dof_control(name, orient, is_move, is_fixed)
            int_marker.controls.append(control)

    def _make_6dof_control(self, name, orientation, is_move, is_fixed):
        '''Creates and returns one component of the 6dof controller.

        Args:
            name (str): Name for hte control
            orientation (Quaternion): How the control should be
                oriented.
            is_move (bool): Looks like whether the marker moves the
                object (?). Currently passed as True for moving markers,
                False for rotating ones.
            is_fixed (bool): Looks like whether position is fixed (?).
                Currently always passed as True.

        Returns:
            InteractiveMarkerControl
        '''
        control = InteractiveMarkerControl()
        control.name = name
        control.orientation = orientation
        control.always_visible = False
        if self.is_control_visible:
            if is_move:
                control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            else:
                control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        else:
            control.interaction_mode = InteractiveMarkerControl.NONE
        if is_fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        return control

    def _make_mesh_marker(self):
        '''Creates and returns a mesh marker.

        Returns:
            Marker
        '''
        mesh = Marker()
        mesh.mesh_use_embedded_materials = False
        mesh.type = Marker.MESH_RESOURCE
        mesh.scale.x = 1.0
        mesh.scale.y = 1.0
        mesh.scale.z = 1.0
        mesh.color = self._get_mesh_marker_color()
        return mesh

    def _get_mesh_marker_color(self):
        '''Gets the color for the mesh marker (thing that looks like a
        gripper) for this step.

        A simple implementation of this will return one color for
        reachable poses, another for unreachable ones. Future
        implementations may provide further visual cues.

        Returns:
            ColorRGBA: The color for the gripper mesh for this step.
        '''
        if self._is_reachable():
            return COLOR_MESH_REACHABLE
        else:
            return COLOR_MESH_UNREACHABLE

    def _make_gripper_marker(self, control, is_hand_open=False):
        '''Makes a gripper marker, adds it to control, returns control.

        Args:
            control (InteractiveMarkerControl): IM Control we're using.
            is_hand_open (bool, optional): Whether the gripper is open.
                Defaults to False (closed).

        Returns:
            InteractiveMarkerControl: The passed control.
        '''
        # Set angle of meshes based on gripper open vs closed.
        angle = ANGLE_GRIPPER_OPEN if is_hand_open else ANGLE_GRIPPER_CLOSED

        # Make transforms in preparation for meshes 1, 2, and 3.
        # NOTE(mbforbes): There are some magic numbers in here that are
        # used a couple times. Seems like a good candidate for
        # refactoring to constants, but I think they're more clear if
        # left in here as (a) they likely won't be changed, and (b) it's
        # easier to understand the computations with them here.
        transform1 = tf.transformations.euler_matrix(0, 0, angle)
        transform1[:3, 3] = [0.07691 - ArmControlMarker._offset, 0.01, 0]
        transform2 = tf.transformations.euler_matrix(0, 0, -angle)
        transform2[:3, 3] = [0.09137, 0.00495, 0]
        t_proximal = transform1
        t_distal = tf.transformations.concatenate_matrices(
            transform1, transform2)

        # Create mesh 1 (palm).
        mesh1 = self._make_mesh_marker()
        mesh1.mesh_resource = STR_GRIPPER_PALM_FILE
        mesh1.pose.position.x = -ArmControlMarker._offset
        mesh1.pose.orientation.w = 1

        # Create mesh 2 (finger).
        mesh2 = self._make_mesh_marker()
        mesh2.mesh_resource = STR_GRIPPER_FINGER_FILE
        mesh2.pose = ArmControlMarker.get_pose_from_transform(t_proximal)

        # Create mesh 3 (fingertip).
        mesh3 = self._make_mesh_marker()
        mesh3.mesh_resource = STR_GRIPPER_FINGERTIP_FILE
        mesh3.pose = ArmControlMarker.get_pose_from_transform(t_distal)

        # Make transforms in preparation for meshes 4 and 5.
        quat = tf.transformations.quaternion_multiply(
            tf.transformations.quaternion_from_euler(numpy.pi, 0, 0),
            tf.transformations.quaternion_from_euler(0, 0, angle)
        )
        transform1 = tf.transformations.quaternion_matrix(quat)
        transform1[:3, 3] = [0.07691 - ArmControlMarker._offset, -0.01, 0]
        transform2 = tf.transformations.euler_matrix(0, 0, -angle)
        transform2[:3, 3] = [0.09137, 0.00495, 0]
        t_proximal = transform1
        t_distal = tf.transformations.concatenate_matrices(
            transform1, transform2)

        # Create mesh 4 (other finger).
        mesh4 = self._make_mesh_marker()
        mesh4.mesh_resource = STR_GRIPPER_FINGER_FILE
        mesh4.pose = ArmControlMarker.get_pose_from_transform(t_proximal)

        # Create mesh 5 (other fingertip).
        mesh5 = self._make_mesh_marker()
        mesh5.mesh_resource = STR_GRIPPER_FINGERTIP_FILE
        mesh5.pose = ArmControlMarker.get_pose_from_transform(t_distal)

        # Append all meshes we made.
        control.markers.append(mesh1)
        control.markers.append(mesh2)
        control.markers.append(mesh3)
        control.markers.append(mesh4)
        control.markers.append(mesh5)

        # Return back the control.
        # TODO(mbforbes): Why do we do this?
        return control
