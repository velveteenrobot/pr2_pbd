# Command.msg: Commands issued through speech or the GUI.

# Constants
# ----------------------------------------------------------------------
# Relax/freeze right/left arm
string RELAX_RIGHT_ARM = relax-right-arm
string RELAX_LEFT_ARM = relax-left-arm
string FREEZE_RIGHT_ARM = freeze-right-arm
string FREEZE_LEFT_ARM = freeze-left-arm

# Open/close right/left hand
string OPEN_RIGHT_HAND = open-right-hand
string OPEN_LEFT_HAND = open-left-hand
string CLOSE_RIGHT_HAND = close-right-hand
string CLOSE_LEFT_HAND = close-left-hand

# General programming
string RECORD_OBJECT_POSE = record-object-pose
string SAVE_POSE = save-pose
string EXECUTE_ACTION = execute-action
string STOP_EXECUTION = stop-execution

# Action navigation
string CREATE_NEW_ACTION = create-new-action
string NEXT_ACTION = next-action
string PREV_ACTION = previous-action

# Step editing
string DELETE_ALL_STEPS = delete-all-steps
string DELETE_LAST_STEP = delete-last-step

# Trajectory
string START_RECORDING_MOTION = start-recording-motion
string STOP_RECORDING_MOTION = stop-recording-motion

# Misc
string TEST_MICROPHONE = test-microphone
string UNRECOGNIZED = unrecognized


# Fields
# ----------------------------------------------------------------------
string command
