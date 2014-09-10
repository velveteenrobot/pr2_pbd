# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_pbd_interaction)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS actionlib arm_navigation_msgs geometry_msgs interactive_markers kinematics_msgs object_manipulation_msgs pr2_arm_kinematics pr2_controllers_msgs pr2_gazebo pr2_gripper_sensor_msgs pr2_interactive_manipulation pr2_interactive_manipulation_frontend rviz pr2_interactive_object_detection pr2_mechanism_controllers pr2_mechanism_msgs pr2_pbd_speech_recognition pr2_social_gaze rospy rostest rqt_gui sound_play std_msgs tabletop_collision_map_processing tf tf_throttle trajectory_filter_server visualization_msgs)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

# Init

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Generate messages and services
add_message_files(
  FILES
  # TODO: List your msg files here
)
add_service_files(
  FILES
  # TODO: List your msg files here
)

# Check launch files
rosbuild_add_roslaunch_check( launch/pbd_frontend.launch )
rosbuild_add_roslaunch_check( launch/pbd_backend_prereqs.launch ROBOT=sim )
rosbuild_add_roslaunch_check( launch/pbd_backend.launch ROBOT=sim )
rosbuild_add_roslaunch_check( launch/pr2_interactive_manipulation_desktop.launch )
rosbuild_add_roslaunch_check( launch/simulated_robot.launch ROBOT=sim ROS_ENV_LOADER=/etc/ros/env.sh )
rosbuild_add_roslaunch_check( launch/pbd_simulation_stack.launch ROBOT=sim ROS_ENV_LOADER=/etc/ros/env.sh )

# Add tests
add_rostest(test/test_endtoend.test)
## Generate added messages and services with any dependencies listed here
generate_messages(
    #TODO DEPENDENCIES geometry_msgs std_msgs
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS actionlib arm_navigation_msgs geometry_msgs interactive_markers kinematics_msgs object_manipulation_msgs pr2_arm_kinematics pr2_controllers_msgs pr2_gazebo pr2_gripper_sensor_msgs pr2_interactive_manipulation pr2_interactive_manipulation_frontend rviz pr2_interactive_object_detection pr2_mechanism_controllers pr2_mechanism_msgs pr2_pbd_speech_recognition pr2_social_gaze rospy rostest rqt_gui sound_play std_msgs tabletop_collision_map_processing tf tf_throttle trajectory_filter_server visualization_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
