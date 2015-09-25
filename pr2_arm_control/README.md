On the robot:
roslaunch pr2_arm_control arm_control_backend.launch

On your computer:
roslaunch pr2_arm_control arm_control_frontend.launch

The default setting is for movement to be "realtime". You can set the parameter "realtime:=False". That will make it so that you have to right-click and say "move gripper to".

You also need my version of moveit_ros: https://github.com/velveteenrobot/moveit_ros

The default branch clear-octomap is the one. It takes a while to build though. 

Now when you publish a point cloud to /head_mount_kinect/depth_registered/points_throttle, it will show up in the octomap. 

You can clear the octomap by calling the 'clear_octomap' service. Otherwise, it will also get cleared when you start the arm_control_node.py node. 
