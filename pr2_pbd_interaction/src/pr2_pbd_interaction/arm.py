'''Interface for controlling one arm.'''

# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import threading
import rospy
import tf
from numpy import array, sign, pi, dot
from numpy.linalg import norm
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from actionlib_msgs.msg import GoalStatus
from actionlib import SimpleActionClient
from pr2_mechanism_msgs.srv import SwitchController
from pr2_mechanism_msgs.srv import SwitchControllerRequest
from pr2_controllers_msgs.msg import JointTrajectoryAction
from pr2_controllers_msgs.msg import JointTrajectoryGoal
from pr2_controllers_msgs.msg import Pr2GripperCommandAction
from pr2_controllers_msgs.msg import Pr2GripperCommandGoal
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, Point, Pose
from pr2_pbd_interaction.msg import GripperState, ArmMode, Side
from pr2_pbd_interaction.world import World
import moveit_commander
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

# ######################################################################
# Module level constants
# ######################################################################

# How much to weight position and rotation differences, respectively, in
# comparing poses for dissimilarity.
W_POS = 1.0
W_ROT = 0.2

# Whether to, by default, automatically release (relax) the arm when a
# user tries to move it while it is holding (frozen).
# TODO(mbforbes): Make this settable in the ROS launch file as param.
DEFAULT_AUTORELEASE_SETTING = False

# What arm mode (frozen/relaxed) to start out with when the system
# begins. One of ArmMode.*.
DEFAULT_ARM_MODE = ArmMode.HOLD
DEFAULT_MOVEMENT_BUFFER_SIZE = 40

# Maximum amount of time to wait for the gripper to finish its movement.
MAX_GRIPPER_WAIT_TIME = 10.0  # seconds

# Default settings for moving grippers.
GRIPPER_OPEN_POSITION = 0.08
GRIPPER_CLOSE_POSITION = 0.00
DEFAULT_GRIPPER_POSITION = GRIPPER_OPEN_POSITION
DEFAULT_GRIPPER_EFFORT = 30.0  # ~30 PSI (I think)
# Whether to wait for the gripper to complete its movement before
# returning.
DEFAULT_GRIPPER_WAIT = False
# Any gripper joint reading above this is considered open.
GRIPPER_OPEN_THRESHOLD = 0.078

# The minimum arm movement that 'counts' as moved.
ARM_MOVEMENT_THRESHOLD = 0.02

# How close an FK-computed pose has to be to the desired pose to use the
# joints (supplied to FK) instead of failing (IK already failed here).
FK_THRESHOLD = 0.02

# The minimum time that an arm needs to remain stable across to be
# counted as stable.
ARM_STABLE_DURATION = rospy.Duration(5.0)  # seconds

# How long to wait once a trajectory goal is published before it should
# be started.
TRAJECTORY_START_DELAY = rospy.Duration(0.1)  # seconds

# How long to allow a trajectory to run for.
TRAJECTORY_ALLOWED_TIME = rospy.Duration(20.0)  # seconds

# What velocity to send to the joint mover to move the arm joints, or
# between trajectory steps.
# NOTE(mbforbes): This is set to 0, but they still move fine. There must
# be some limits / defaults that are activated when 0 is passed.
MOVE_TO_JOINTS_VELOCITY = 0
TRAJ_VELOCITY = 0

# Joint / part names
GRIPPER_JOINT_POSTFIX = '_gripper_joint'
EE_POSTFIX = '_wrist_roll_link'
ARM_JOINTS = [
    '_shoulder_pan_joint',
    '_shoulder_lift_joint',
    '_upper_arm_roll_joint',
    '_elbow_flex_joint',
    '_forearm_roll_joint',
    '_wrist_flex_joint',
    '_wrist_roll_joint'
]

# IK settings
IK_REQUEST_TIMEOUT = 4.0  # seconds

# PR2
PR2_SERVICE_PREFIX = 'pr2_'
BASE_LINK = 'base_link'


# ######################################################################
# Classes
# ######################################################################

class Arm:
    ''' Interfacing with one arm for controlling mode and action
    execution.'''

    # Note that auto-release also activates auto-hold (if the arm isn't
    # held, but is stable, it will switch to holding mode). See the
    # update method for more info.
    _is_autorelease_on = DEFAULT_AUTORELEASE_SETTING

    def __init__(self, arm_index):
        '''
        Args:
            arm_index (int): Side.RIGHT or Side.LEFT
        '''
        # Set initial state to retrieve arm-specific strings.
        self.arm_index = arm_index

        # Extract these locals for ease of use.
        side = self._side()
        side_prefix = self._side_prefix()

        # Set state
        self.arm_mode = DEFAULT_ARM_MODE
        self.gripper_state = None
        self.gripper_joint_name = side_prefix + GRIPPER_JOINT_POSTFIX
        self.ee_name = side_prefix + EE_POSTFIX
        self.joint_names = [side_prefix + joint for joint in ARM_JOINTS]

        self.all_joint_names = []
        self.all_joint_poses = []

        self.last_ee_pose = None
        self.movement_buffer_size = DEFAULT_MOVEMENT_BUFFER_SIZE
        self.last_unstable_time = rospy.Time.now()
        self.arm_movement = []

        rospy.loginfo('Initializing ' + side + ' arm.')

        self.lock = threading.Lock()
        rospy.Subscriber('joint_states', JointState, self._joint_states_cb)

        switch_controller = 'pr2_controller_manager/switch_controller'
        rospy.wait_for_service(switch_controller)
        self.switch_service = rospy.ServiceProxy(
            switch_controller, SwitchController)
        rospy.loginfo(
            'Got response form the switch controller for ' + side + ' arm.')

        # Create a trajectory action client
        traj_controller_name = (
            side_prefix + '_arm_controller/joint_trajectory_action')
        self.traj_action_client = SimpleActionClient(
            traj_controller_name, JointTrajectoryAction)
        self.traj_action_client.wait_for_server()
        rospy.loginfo(
            'Got response form trajectory action server for ' + side + ' arm.')

        # Set up Inversse Kinematics
        self.ik_srv = None
        self.ik_request = None
        self.ik_joints = None
        self.ik_limits = None
        self._setup_ik()

        # Set up gripper controller.
        gripper_name = side_prefix + '_gripper_controller/gripper_action'
        self.gripper_client = SimpleActionClient(
            gripper_name, Pr2GripperCommandAction)
        self.gripper_client.wait_for_server()
        rospy.loginfo('Got response form gripper server for ' + side + ' arm.')

    # ##################################################################
    # Static methods: Public (API)
    # ##################################################################

    @staticmethod
    def get_distance_bw_poses(pose0, pose1):
        '''Returns the dissimilarity between two end-effector poses.

        Specifically, returns the most dissimilar aspect of distance
        between two poses, either the position difference or rotation
        difference, each weighted by W_POS and W_ROT, respectively.

        Args:
            pose0 (Pose)
            pose1 (Pose)

        Returns:
            float: The dissimilarity amount of the most dissimilar
                aspect between two poses, either their position or
                rotation.
        '''
        # Extract raw positions and orientations as numpy arrays.
        pos0 = array((pose0.position.x, pose0.position.y, pose0.position.z))
        pos1 = array((pose1.position.x, pose1.position.y, pose1.position.z))
        rot0 = array((pose0.orientation.x, pose0.orientation.y,
                      pose0.orientation.z, pose0.orientation.w))
        rot1 = array((pose1.orientation.x, pose1.orientation.y,
                      pose1.orientation.z, pose1.orientation.w))

        # Weight products.
        pos_dist = W_POS * norm(pos0 - pos1)
        rot_dist = W_ROT * (1 - dot(rot0, rot1))

        # Return max dissimilarity aspect.
        if (pos_dist > rot_dist):
            dist = pos_dist
        else:
            dist = rot_dist
        return dist

    # ##################################################################
    # Instance methods: Public (API)
    # ##################################################################

    def is_gripper_moving(self):
        '''Returns whether the gripper is in the process of
        opening/closing.

        Returns:
            bool
        '''
        return (self.gripper_client.get_state() == GoalStatus.ACTIVE or
                self.gripper_client.get_state() == GoalStatus.PENDING)

    def is_gripper_at_goal(self):
        '''Returns whether the gripper has reached its goal.

        Returns:
            bool
        '''
        return self.gripper_client.get_state() == GoalStatus.SUCCEEDED

    def get_gripper_state(self):
        '''Returns current gripper state.

        Returns:
            int: GripperState.OPEN or GripperState.CLOSED
        '''
        return self.gripper_state

    def check_gripper_state(self, joint_name=None):
        '''Checks gripper state at the hardware level.

        Sets the result internally (to self.gripper_state) rather than
        return it.

        Args:
            joint_name (str, optional): The name of the joint to check.
                Defaults to None (which in turn defaults to the gripper
                joint of this arm's side).
        '''
        if joint_name is None:
            joint_name = self.gripper_joint_name
        gripper_pos = self.get_joint_positions([joint_name])
        if gripper_pos != []:
            self.gripper_state = (
                GripperState.OPEN if gripper_pos[0] > GRIPPER_OPEN_THRESHOLD
                else GripperState.CLOSED)
        else:
            rospy.logwarn('Could not update the gripper state.')

    def open_gripper(
            self,
            pos=GRIPPER_OPEN_POSITION,
            eff=DEFAULT_GRIPPER_EFFORT,
            wait=DEFAULT_GRIPPER_WAIT):
        '''Opens gripper.

        Args:
            pos (float, optional): The position ofthe gripper to move
                to. Close is GRIPPER_CLOSE_POSITION, open is
                GRIPPER_OPEN_POSITION. Defaults to
                GRIPPER_OPEN_POSITION.
            eff (float, optional): How much effort to exert when moving
                the gripper to its position. Units are in PSI (I think).
                Defaults to DEFAULT_GRIPPER_EFFORT.
            wait (bool, optional): Whether to wait for the gripper to
                complete its goal before returning. Defaults to
                DEFAULT_GRIPPER_WAIT.
        '''
        self._send_gripper_command(pos, eff, wait)
        self.gripper_state = GripperState.OPEN

    def close_gripper(
            self,
            pos=GRIPPER_CLOSE_POSITION,
            eff=DEFAULT_GRIPPER_EFFORT,
            wait=DEFAULT_GRIPPER_WAIT):
        '''Closes gripper.

        Args:
            pos (float, optional): The position ofthe gripper to move
                to. Close is GRIPPER_CLOSE_POSITION, open is
                GRIPPER_OPEN_POSITION. Defaults to
                GRIPPER_CLOSE_POSITION.
            eff (float, optional): How much effort to exert when moving
                the gripper to its position. Units are in PSI (I think).
                Defaults to DEFAULT_GRIPPER_EFFORT.
            wait (bool, optional): Whether to wait for the gripper to
                complete its goal before returning. Defaults to
                DEFAULT_GRIPPER_WAIT.
        '''
        self._send_gripper_command(pos, eff, wait)
        self.gripper_state = GripperState.CLOSED

    def set_gripper(self, gripper_state):
        '''Sets gripper to the desired state.

        Args:
            gripper_state (int): GripperState.OPEN or
                GripperState.CLOSED
        '''
        if gripper_state == GripperState.CLOSED:
            self.close_gripper()
        else:
            # gripper_state == GripperState.OPEN
            self.open_gripper()

    def execute_joint_traj(self, joint_trajectory, timing):
        '''Moves the arm through the joint sequence.

        Args:
            joint_trajectory (ArmState[]): List of ArmStates to move to
                (either for the right or left arm).
            timing (duration[])
        '''
        # First, do filtering on the trajectory to fix the velocities.
        trajectory = JointTrajectory()

        # Initialize the server.
        # When to start the trajectory: TRAJECTORY_START_DELAY seconds from now
        trajectory.header.stamp = rospy.Time.now() + TRAJECTORY_START_DELAY
        trajectory.joint_names = self.joint_names

        # Add all frames of the trajectory as way points.
        for i in range(len(timing)):
            positions = joint_trajectory[i].joint_pose
            velocities = [TRAJ_VELOCITY] * len(positions)
            # Add frames to the trajectory
            trajectory.points.append(JointTrajectoryPoint(
                positions=positions,
                velocities=velocities,
                time_from_start=timing[i])
            )

        output = self.filter_service(
            trajectory=trajectory, allowed_time=TRAJECTORY_ALLOWED_TIME)
        rospy.loginfo(
            '\tTrajectory for ' + self._side() + ' arm has been filtered.')

        # TODO(mcakmak): Check output.error_code.
        traj_goal = JointTrajectoryGoal()
        traj_goal.trajectory = output.trajectory
        traj_goal.trajectory.header.stamp = (
            rospy.Time.now() + TRAJECTORY_START_DELAY)
        traj_goal.trajectory.joint_names = self.joint_names

        # Sends the goal to the trajectory server
        self.traj_action_client.send_goal(traj_goal)

    def move_to_joints(self, joints, time_to_joint):
        '''Moves the arm to the desired joints.

        Args:
            joints ([float]): Seven-element list of arm joint positions
                to move to.
            time_to_joint (float): How long (in seconds) to allow for
                moving arm_index to the next arm state.
        '''
        # Setup the goal
        traj_goal = JointTrajectoryGoal()
        traj_goal.trajectory.header.stamp = (
            rospy.Time.now() + TRAJECTORY_START_DELAY)
        traj_goal.trajectory.joint_names = self.joint_names
        velocities = [MOVE_TO_JOINTS_VELOCITY] * len(joints)
        traj_goal.trajectory.points.append(JointTrajectoryPoint(
            positions=joints,
            velocities=velocities,
            time_from_start=rospy.Duration(time_to_joint))
        )

        # Send the goal to the server.
        self.traj_action_client.send_goal(traj_goal)

    def is_executing(self):
        '''Whether or not there is an ongoing action execution on the arm.

        Returns:
            bool
        '''
        return (self.traj_action_client.get_state() == GoalStatus.ACTIVE
                or self.traj_action_client.get_state() == GoalStatus.PENDING)

    def is_successful(self):
        '''Returns whether the execution succeeded.

        Returns:
            bool
        '''
        return self.traj_action_client.get_state() == GoalStatus.SUCCEEDED

    def get_ik_for_ee(self, ee_pose, seed):
        ''' Finds the IK solution for given end effector pose.

        Note that though seed is not explicitly allowed to be None in
        this method, it often is in code that this calls. If None is
        passed in for the seed, None may be returned.

        Args:
            ee_pose (Pose): Pose to solve IK for.
            seed ([float]): Seven-element list of arm joint positions.

        Returns:
            [float]: Seven-element list of arm joint positions. If seed
                passed in is None, and no IK solution is found, None
                could be returned.
        '''
        joints = self._solve_ik(ee_pose, seed)
        # If our seed did not work, try once again with the default
        # seed.
        if joints is None:
            rospy.logdebug(
                'Could not find IK solution with preferred seed, will try ' +
                'default seed.')
            joints = self._solve_ik(ee_pose)

        if joints is None:
            rospy.logdebug('IK out of bounds, considering the seed directly.')
            # IK failed, but let's see if FK with the passed seed will
            # give us a pose close enough to the ee_pose that it's
            # usable.
            fk_pose = self.get_fk_for_joints(seed)
            if fk_pose is not None:
                if Arm.get_distance_bw_poses(ee_pose, fk_pose) < FK_THRESHOLD:
                    joints = seed
                    rospy.logdebug(
                        'IK out of bounds, but FK close; using seed.')
                # We don't report it if FK isn't close as this will
                # happen for about all relative but unreachable poses.
            else:
                rospy.logdebug(
                    'FK failed and returned None for seed: ' + str(seed))
        else:
            rollover = array((array(joints) - array(seed)) / pi, int)
            joints -= ((rollover + (sign(rollover) + 1) / 2) / 2) * 2 * pi

        return joints

    def get_fk_for_joints(self, joints):
        '''Finds the FK solution (EE pose) for given joint positions.

        Args:
            joints ([float]) Seven-element list of arm joint positions.

        Returns:
            Pose|None: The arm's pose when its joints are at the
                specified values, or None if there was a problem finding
                an FK solution
        '''
        self.fk_request.robot_state.joint_state.position = joints
        pose = None
        try:
            resp = self.fk_srv(self.fk_request)
            pose_idx = resp.fk_link_names.index(self.ee_name)
            pose = resp.pose_stamped[pose_idx].pose
        except rospy.ServiceException:
            rospy.logwarn(
                'There was an error with the FK request for joints: ' +
                str(joints))
        finally:
            return pose

    def reset_movement_history(self):
        '''Clears the saved history of arm movements.'''
        self.last_unstable_time = rospy.Time.now()
        self.arm_movement = []

    def get_movement(self):
        '''Returns cumulative movement in recent history.

        Returns:
            float
        '''
        return sum(self.arm_movement)

    def get_side_str(self):
        '''
        Returns a string representing the side this arm is on (either
            'right' or 'left').

        Returns:
            str
        '''
        return self._side()

    def get_ee_state(self, ref_frame=BASE_LINK):
        '''Returns the current end effector pose for the arm.

        Args:
            ref_frame (str, optional): The reference frame for the end
                effector pose to use. Defaults to BASE_LINK.

        Returns:
            Pose|None: Pose if success, None if there was a failure in
                looking up the transform to ref_frame.
        '''
        try:
            # Transform the position/orientation of the current pose
            # into the desired reference frame.
            time = World.tf_listener.getLatestCommonTime(
                ref_frame, self.ee_name)
            position, orientation = World.tf_listener.lookupTransform(
                ref_frame, self.ee_name, time)

            # Construct and return the Pose.
            ee_pose = Pose()
            ee_pose.position = Point(position[0], position[1], position[2])
            ee_pose.orientation = Quaternion(
                orientation[0], orientation[1], orientation[2], orientation[3])
            return ee_pose
        except (
                tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            rospy.logwarn('Something wrong with transform request.')
            return None

    def get_joint_positions(self, joint_names=None):
        '''Returns position for the requested or all arm joints.

        Args:
            joint_names ([str], optional): A list of all requested joint
                names. Defaults to None (in which case all seven arm
                joint names will be used).

        Returns:
            [float]: An array of the position of each joint requested in
                joint_names, or all seven arm joints if joint_names was
                not passed in (or was passed as None).
        '''
        if joint_names is None:
            joint_names = self.joint_names

        if self.all_joint_names == []:
            rospy.logerr("No robot_state messages received.")
            return []

        positions = []
        self.lock.acquire()
        for joint_name in joint_names:
            if joint_name in self.all_joint_names:
                index = self.all_joint_names.index(joint_name)
                position = self.all_joint_poses[index]
                positions.append(position)
            else:
                rospy.logerr("Joint %s not found!", joint_name)
        self.lock.release()
        return positions

    def set_mode(self, mode):
        '''Releases or holds the arm by turning the controller on/off.

        Args:
            mode (int): ArmMode.HOLD or ArmMode.RELEASE
        '''
        side = self._side()
        controller_name = self._side_prefix() + '_arm_controller'
        if mode == ArmMode.RELEASE:
            start_controllers = []
            stop_controllers = [controller_name]
            rospy.loginfo('Switching ' + side + ' arm to the kinesthetic mode')
        elif mode == ArmMode.HOLD:
            start_controllers = [controller_name]
            stop_controllers = []
            rospy.loginfo('Switching ' + side + ' to the joint-control mode')
        else:
            rospy.logwarn(
                'Unknown mode: ' + str(mode) + '. Keeping the current mode.')
            return

        try:
            self.switch_service(
                start_controllers,
                stop_controllers,
                SwitchControllerRequest.BEST_EFFORT  # strictness
            )
            self.arm_mode = mode
        except rospy.ServiceException:
            rospy.logerr("Arm mode switch service did not process request")

    def get_mode(self):
        '''Returns the current arm mode (released/holding).

        Returns:
            int: ArmMode.RELEASE or ArmMode.HOLD

        '''
        return self.arm_mode

    def update(self, is_executing):
        ''' Periodical update for one arm.

        Args:
            is_executing (bool): Whether there is an ongoing action
                execution.
        '''
        # Track EE poses.
        ee_pose = self.get_ee_state()
        if ee_pose is not None and self.last_ee_pose is not None:
            self._record_arm_movement(Arm.get_distance_bw_poses(
                ee_pose, self.last_ee_pose))
        self.last_ee_pose = ee_pose

        # Activate auto-relese if it's enabled and needed.
        if not is_executing and Arm._is_autorelease_on:
            if self._is_arm_moved_while_holding():
                rospy.loginfo('Automatically releasing arm.')
                self.set_mode(ArmMode.RELEASE)

            if self._is_arm_stable_while_released():
                rospy.loginfo('Automatically holding arm.')
                self.set_mode(ArmMode.HOLD)

    # ##################################################################
    # Instance methods: Internal ("private")
    # ##################################################################

    def _joint_states_cb(self, msg):
        '''Callback function that saves the joint positions when a
        joint_states message is received.

        Args:
            msg (JointState)
        '''
        # We just cache this internally.
        self.lock.acquire()
        self.all_joint_names = msg.name
        self.all_joint_poses = msg.position
        self.lock.release()

    def _setup_ik(self):
        '''Sets up services for inverse kinematics.'''
        ik_srv_name = '/compute_ik'
        rospy.loginfo('IK info service has responded for '
                      + self._side() + ' arm.')
        rospy.wait_for_service(ik_srv_name)
        self.ik_srv = rospy.ServiceProxy(ik_srv_name,
                                         GetPositionIK, persistent=True)
        rospy.loginfo('IK service has responded for ' + self._side() + ' arm.')

        robot = moveit_commander.RobotCommander()
        # Set up common parts of an IK request
        self.ik_request = GetPositionIKRequest()
        request = self.ik_request.ik_request
        request.timeout = rospy.Duration(4)
        group_name = self._side() + '_arm'
        request.group_name = group_name
        self.ik_joints = self.joint_names
        self.ik_limits = [robot.get_joint(x).bounds() for x in self.ik_joints]
        request.ik_link_name = self.ee_name
        request.pose_stamped.header.frame_id = 'base_link'
        request.robot_state.joint_state.name = self.ik_joints
        request.robot_state.joint_state.position = [0] * len(self.joint_names)

    def _side(self):
        '''Returns the string 'right' or 'left' depending on arm side.

        Returns:
            str: 'right' or 'left'
        '''
        return 'right' if self.arm_index == Side.RIGHT else 'left'

    def _side_prefix(self):
        ''' Returns the string 'r' or 'l' depending on arm side.

        Returns:
            str: 'r' or 'l'
        '''
        return self._side()[0]

    def _solve_ik(self, ee_pose, seed=None):
        '''Gets the IK solution for end effector pose.

        Args:
            ee_pose (Pose): The pose to solve IK for.
            seed ([float], optional): A seven-element list of floats of
                all arm joint positions. Defaults to None, in which case
                the midpoint of all joint ranges will be used.

        Returns:
            [float]|None: A seven-element list of floats of all arm
                joint positions to reach ee_pose, or None if no IK
                solution was found.
        '''
        self.ik_request.ik_request.pose_stamped.pose = ee_pose

        # If no seed is specified for IK search, start search at
        # midpoint (i.e. craft a default seed).
        if seed is None:
            seed = []
            for i in range(0, len(self.ik_joints)):
                seed.append((self.ik_limits[i].min_position +
                             self.ik_limits[i].max_position) / 2.0)
        self.ik_request.ik_request.ik_seed_state.joint_state.position = seed

        # Send off the IK request.
        try:
            rospy.logdebug('Sending IK request.')
            response = self.ik_srv(self.ik_request)
            if response.error_code.val == response.error_code.SUCCESS:
                return response.solution.joint_state.position
            else:
                return None
        except rospy.ServiceException:
            rospy.logerr('Exception while getting the IK solution.')
            return None

    def _send_gripper_command(
            self,
            pos=DEFAULT_GRIPPER_POSITION,
            eff=DEFAULT_GRIPPER_EFFORT,
            wait=DEFAULT_GRIPPER_WAIT):
        '''Sets the position of the gripper.

        Args:
            pos (float, optional): The position ofthe gripper to move
                to. Close is GRIPPER_CLOSE_POSITION, open is
                GRIPPER_OPEN_POSITION. Defaults to
                DEFAULT_GRIPPER_POSITION.
            eff (float, optional): How much effort to exert when moving
                the gripper to its position. Units are in PSI (I think).
                Defaults to DEFAULT_GRIPPER_EFFORT.
            wait (bool, optional): Whether to wait for the gripper to
                complete its goal before returning. Defaults to
                DEFAULT_GRIPPER_WAIT.
        '''
        goal = Pr2GripperCommandGoal()
        goal.command.position = pos
        goal.command.max_effort = eff
        self.gripper_client.send_goal(goal)
        if wait:
            self.gripper_client.wait_for_result(
                rospy.Duration(MAX_GRIPPER_WAIT_TIME))

    def _record_arm_movement(self, reading):
        '''Records the sensed arm movement.

        Args:
            reading (float): The most recent sensed arm movement.
        '''
        self.arm_movement = [reading] + self.arm_movement
        if len(self.arm_movement) > self.movement_buffer_size:
            self.arm_movement = self.arm_movement[:self.movement_buffer_size]

    def _is_arm_moved_while_holding(self):
        '''Returns whether the user is trying to move the arm while it
        is stiff.

        Returns:
            bool
        '''
        return (
            self.get_mode() == ArmMode.HOLD
            and len(self.arm_movement) == self.movement_buffer_size
            and self.get_movement() > ARM_MOVEMENT_THRESHOLD)

    def _is_arm_stable_while_released(self):
        '''Returns whether the arm has been stable while being
        released.

        Returns:
            bool
        '''
        is_arm_stable = self.get_movement() < ARM_MOVEMENT_THRESHOLD
        if not is_arm_stable or self.get_mode() == ArmMode.HOLD:
            self.last_unstable_time = rospy.Time.now()
            return False
        else:
            return (
                rospy.Time.now() - self.last_unstable_time >
                ARM_STABLE_DURATION)
