On the robot:
roslaunch pr2_arm_control arm_control_backend.launch

On your computer:
roslaunch pr2_arm_control arm_control_frontend.launch

The default setting is for movement to be "realtime". You can set the parameter "realtime:=False". That will make it so that you have to right-click and say "move gripper to".
