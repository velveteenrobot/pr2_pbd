'''Represents a single 3D marker (in RViz) for visualizing the steps of
an action.'''

# ######################################################################
# Imports
# ######################################################################

import rospy
import numpy
from geometry_msgs.msg import Quaternion, Vector3, Point, Pose, PoseStamped
from std_msgs.msg import Header, ColorRGBA
from sensor_msgs.msg import JointState
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
# Constants
# ######################################################################

DEFAULT_OFFSET = 0.09
COLOR_MESH_REACHABLE = ColorRGBA(0.5, 0.5, 0.5, 0.6)
COLOR_MESH_UNREACHABLE = ColorRGBA(1.0, 0.0, 0.0, 0.6)
ANGLE_GRIPPER_OPEN = 28 * numpy.pi / 180.0
ANGLE_GRIPPER_CLOSED = 0.0
STR_MESH_GRIPPER_FOLDER = 'package://pr2_description/meshes/gripper_v0/'
STR_GRIPPER_PALM_FILE = STR_MESH_GRIPPER_FOLDER + 'gripper_palm.dae'
STR_GRIPPER_FINGER_FILE = STR_MESH_GRIPPER_FOLDER + 'l_finger.dae'
STR_GRIPPER_FINGERTIP_FILE = STR_MESH_GRIPPER_FOLDER + 'l_finger_tip.dae'
INT_MARKER_SCALE = 0.2
GRIPPER_MARKER_SCALE = 1.05
REF_FRAME = 'base_link'

# ######################################################################
# Class 
# ######################################################################

class ArmControlMarker:
    '''Marker for visualizing the steps of an action.'''

    _im_server = None

    def __init__(self, arm):
        '''
        Args:
            arm (Arm): Arm object for the right or left PR2 arm.
        '''
        if ArmControlMarker._im_server is None:
            im_server = InteractiveMarkerServer('interactive_arm_control')
            ArmControlMarker._im_server = im_server
        self._move_upper = False
        self._arm = arm
        self._is_control_visible = False
        self._menu_handler = None
        self._prev_is_reachable = None
        self._pose = self._arm.get_ee_state()
        self._arm_letter = ['r', 'l']
        self._tf_listener = tf.TransformListener()
        self._click_sub = rospy.Subscriber('/clicked_point_normal', PoseStamped, self.clicked_point_cb)
        
        self._pose_upper = self.get_arm_roll_pose()
        self._lock = threading.Lock()

    def clicked_point_cb(self, pose_stamped):
        pose_stamped.header.stamp = rospy.Time(0)
        new_pose = self._tf_listener.transformPose(REF_FRAME, pose_stamped)
#        print(new_pose)
#        new_pose.pose.orientation.x = -1*new_pose.pose.orientation.x
#        new_pose.pose.orientation.y = -1*new_pose.pose.orientation.y
#        new_pose.pose.orientation.z = -1*new_pose.pose.orientation.z
#        new_pose.pose.orientation.w = 1*new_pose.pose.orientation.w
#        print(new_pose)
        self.set_new_pose(new_pose.pose)

    def get_arm_roll_pose(self):
        pose = PoseStamped()
        pose.pose.orientation.w = 0
        pose.header.frame_id = self._arm_letter[self._arm.arm_index] + '_upper_arm_roll_link'
        self._tf_listener.waitForTransform(self._arm_letter[self._arm.arm_index] + '_upper_arm_roll_link', self._arm_letter[self._arm.arm_index] + '_shoulder_lift_link', rospy.Time(0), rospy.Duration(4.0))
        new_pose = self._tf_listener.transformPose(self._arm_letter[self._arm.arm_index] + '_shoulder_lift_link', pose)
        return new_pose.pose


    def update(self, realtime):

        self._menu_handler = MenuHandler()

        # Inset main menu entries.
        if not realtime:
            self._menu_handler.insert(
                'Move gripper here', callback=self.move_to_cb)
            self._menu_handler.insert(
                'Move marker to current gripper pose',
                callback=self.move_pose_to_cb)

        if self._is_hand_open():
            self._menu_handler.insert(
                'Close gripper',
                callback=self.close_gripper_cb)
        else:
            self._menu_handler.insert(
                'Open gripper',
                callback=self.open_gripper_cb)

        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.BUTTON
        menu_control.always_visible = True
        frame_id = REF_FRAME
        pose = self.get_pose()

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

        self._menu_handler.apply(ArmControlMarker._im_server,
            self._get_name())
        ArmControlMarker._im_server.applyChanges()

        pose = self.get_pose_upper()
        #self.set_new_pose_upper(pose)

        # Make and add interactive marker.
        int_marker_upper = InteractiveMarker()
        int_marker_upper.name = self._get_name() + '_upper'
        int_marker_upper.header.frame_id = self._arm_letter[self._arm.arm_index] + '_shoulder_lift_link'
        int_marker_upper.pose = pose
        int_marker_upper.scale = INT_MARKER_SCALE * 2
        self._add_1dof_marker(int_marker_upper, True)
        # int_marker_upper.controls.append(menu_control)
        ArmControlMarker._im_server.insert(
            int_marker_upper, self.marker_feedback_cb_upper)

        self._menu_handler.apply(ArmControlMarker._im_server,
            self._get_name() + '_upper')
        ArmControlMarker._im_server.applyChanges()


    def reset(self):
        self.set_new_pose(self._arm.get_ee_state(), is_offset=True)  

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
                by (scales DEFAULT_OFFSET). Defaults to 1.

        Returns:
            Pose: The offset pose.
        '''
        transform = ArmControlMarker.get_matrix_from_pose(pose)
        offset_array = [constant * DEFAULT_OFFSET, 0, 0]
        offset_transform = tf.transformations.translation_matrix(offset_array)
        hand_transform = tf.transformations.concatenate_matrices(
            transform, offset_transform)
        return ArmControlMarker.get_pose_from_transform(hand_transform)

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

    def set_new_pose_upper(self, new_pose, is_offset=False):
        '''Changes the pose of the action step to new_pose.

        Args:
            new_pose (Pose)
        '''
        self._lock.acquire()
        if is_offset:
            self._pose_upper = new_pose
        else:
            self._pose_upper = ArmControlMarker._offset_pose(new_pose, -1)
        self._lock.release()

    def set_new_pose(self, new_pose, is_offset=False):
        '''Changes the pose of the action step to new_pose.

        Args:
            new_pose (Pose)
        '''
        self._lock.acquire()
        if is_offset:
            self._pose = new_pose
        else:
            self._pose = ArmControlMarker._offset_pose(new_pose, -1)
        self._lock.release()

    @staticmethod
    def copy_pose(pose):
        copy = Pose(
            Point(pose.position.x, pose.position.y, pose.position.z),
            Quaternion(pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w)
        )
        return copy

    def get_pose(self):
        '''Returns the pose of the action step.

        Returns:
            Pose
        '''
        self._lock.acquire()
        pose = ArmControlMarker.copy_pose(self._pose)
        self._lock.release()
        return ArmControlMarker._offset_pose(pose)

    def get_pose_upper(self):
        '''Returns the pose of the action step.

        Returns:
            Pose
        '''
        self._lock.acquire()
        pose = ArmControlMarker.copy_pose(self._pose_upper)
        self._lock.release()
        return ArmControlMarker._offset_pose(pose)

    def marker_feedback_cb_upper(self, feedback):
        '''Callback for when an event occurs on the marker.

        Args:
            feedback (InteractiveMarkerFeedback)
        '''
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.set_new_pose_upper(feedback.pose)
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self._arm.cancel_move_to_joints()
            self._pose_upper = self.get_arm_roll_pose()
            self.update(None)
            self._move_upper = False

        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            self._move_upper = True
        # self.move_to_cb_upper(None)

    def marker_feedback_cb(self, feedback):
        '''Callback for when an event occurs on the marker.

        Args:
            feedback (InteractiveMarkerFeedback)
        '''
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.set_new_pose(feedback.pose)
        elif feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            # Set the visibility of the 6DOF controller.
            # This happens a ton, and doesn't need to be logged like
            # normal events (e.g. clicking on most marker controls
            # fires here).
            rospy.logdebug('Changing visibility of the pose controls.')
            self._is_control_visible = not self._is_control_visible
        else:
            # This happens a ton, and doesn't need to be logged like
            # normal events (e.g. clicking on most marker controls
            # fires here).
            rospy.logdebug('Unknown event: ' + str(feedback.event_type))



    def open_gripper_cb(self, __):
        self._arm.open_gripper()

    def close_gripper_cb(self, __):
        self._arm.close_gripper()

    def move_to_cb_upper(self, __):
        '''Callback for when moving to a pose is requested.

        Args:
            __ (???): Unused
        '''
        if not self._move_upper:
            return
        self._arm.cancel_old_goals()
        rospy.loginfo("Moving upper arm!!!")
        self._lock.acquire()
        pose = ArmControlMarker.copy_pose(self._pose_upper)
        pose_2 = ArmControlMarker.copy_pose(self._pose)
        self._lock.release()

        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        target_joints = self._arm.get_ik_for_ee(
                pose_2, self._arm.get_joint_state())
        # for i in range(len(target_joints)):
        #     target_joints[i] = 9999
        if not target_joints is None:
            print target_joints[2]
            print roll
            roll_diff = roll - target_joints[2]
            if roll_diff > 0.15:
                roll_diff = 0.15
            elif roll_diff < -0.15:
                roll_diff = -0.15
            target_joints[2] += roll_diff
        side_str = self._arm.side()

        if target_joints is not None:
            time_to_pose = self._arm.get_time_to_pose(self.get_pose())

            thread = threading.Thread(
                group=None,
                target=self._arm.move_to_joints,
                args=(target_joints, time_to_pose),
                name='move_to_arm_state_thread'
            )
            thread.start()
            rospy.sleep(0.1)

            # Log
            
            rospy.loginfo('Started thread to move ' + side_str + ' arm.')
        else:
            rospy.loginfo('Will not move ' + side_str + ' arm; unreachable.')


        self.move_to_cb(None)

    def move_to_cb(self, __):
        '''Callback for when moving to a pose is requested.

        Args:
            __ (???): Unused
        '''

        self._arm.cancel_old_goals()
        self._lock.acquire()
        pose = ArmControlMarker.copy_pose(self._pose)
        self._lock.release()

        target_joints = self._arm.get_ik_for_ee(
                pose, self._arm.get_joint_state())
        side_str = self._arm.side()

        if target_joints is not None:
            time_to_pose = self._arm.get_time_to_pose(self.get_pose())

            self.thread = threading.Thread(
                group=None,
                target=self._arm.move_to_joints,
                args=(target_joints, time_to_pose),
                name='move_to_arm_state_thread'
            )
            self.thread.start()

            # Log
            
            rospy.loginfo('Started thread to move ' + side_str + ' arm.')
        else:
            rospy.loginfo('Will not move ' + side_str + ' arm; unreachable.')

    def move_pose_to_cb(self, __):
        '''Callback for when a pose change to current is requested.

        Args:
            __ (???): Unused

        '''
        self.reset()

    def _is_reachable(self):
        '''Checks and returns whether there is an IK solution for this
        action step.

        Returns:
            bool: Whether this action step is reachable.
        '''

        self._lock.acquire()
        pose = ArmControlMarker.copy_pose(self._pose)
        self._lock.release()

        target_joints = self._arm.get_ik_for_ee(
                pose, self._arm.get_joint_state())


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

    def _is_hand_open(self):
        '''Returns whether the gripper is open for this action step.

        Returns:
            bool
        '''
        return self._arm.get_gripper_state() == GripperState.OPEN

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
            control = self._make_6dof_control(
                name, orient, is_move, is_fixed)
            int_marker.controls.append(control)

    def _add_1dof_marker(self, int_marker, is_fixed):
        '''Adds a 6 DoF control marker to the interactive marker.

        Args:
            int_marker (InteractiveMarker)
            is_fixed (bool): Looks like whether position is fixed (?).
                Currently only passed as True.
        '''
        # Each entry in options is (name, orientation, is_move)
        options = [
            ('rotate_x', Quaternion(1, 0, 0, 1), False)
        ]
        for opt in options:
            name, orient, is_move = opt
            control = self._make_6dof_control(
                name, orient, is_move, is_fixed)
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
        if self._is_control_visible:
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
        mesh.scale.x = GRIPPER_MARKER_SCALE
        mesh.scale.y = GRIPPER_MARKER_SCALE
        mesh.scale.z = GRIPPER_MARKER_SCALE
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
        if is_hand_open:
            angle = ANGLE_GRIPPER_OPEN
        else:
            msg = rospy.wait_for_message("joint_states", JointState)
            idx = msg.name.index(self._arm_letter[self._arm.arm_index] + '_gripper_joint')
            angle = 350*msg.position[idx]*numpy.pi/180  
        # Make transforms in preparation for meshes 1, 2, and 3.
        # NOTE(mbforbes): There are some magic numbers in here that are
        # used a couple times. Seems like a good candidate for
        # refactoring to constants, but I think they're more clear if
        # left in here as (a) they likely won't be changed, and (b) it's
        # easier to understand the computations with them here.
        transform1 = tf.transformations.euler_matrix(0, 0, angle)
        transform1[:3, 3] = [0.07691 - DEFAULT_OFFSET, 0.01, 0]
        transform2 = tf.transformations.euler_matrix(0, 0, -angle)
        transform2[:3, 3] = [0.09137, 0.00495, 0]
        t_proximal = transform1
        t_distal = tf.transformations.concatenate_matrices(
            transform1, transform2)

        # Create mesh 1 (palm).
        mesh1 = self._make_mesh_marker()
        mesh1.mesh_resource = STR_GRIPPER_PALM_FILE
        mesh1.pose.position.x = -DEFAULT_OFFSET
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
        transform1[:3, 3] = [0.07691 - DEFAULT_OFFSET, -0.01, 0]
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
