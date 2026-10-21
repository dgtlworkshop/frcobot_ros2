# From Collaborative Robot Controller Communication Command Protocol User Manual.pdf

# Response success
uint16 SUCCESS=0
# Check the number of interface parameters
uint16 PARAM_COUNT=3
# Check the type or range of parameter values
uint16 PARAM_VALUE=4
# Check the TPD timer parameter Settings
uint16 TPD_TIMER_START=5
# Check the TPD timer parameter Settings
uint16 TPD_TIMER_CLOSE=6
# Check the TPD file name
uint16 TPD_FILE_CREATE=7
# Check whether the TPD trajectory file exists or whether the trajectory name is correct
uint16 TPD_FILE_MISSING=8
# Check that the TPD track name is correct
uint16 TPD_FILE_NAME_SEND=9
# Check that the TPD file is correct
uint16 TPD_FILE_CONTENT_SEND=10
# Check the contents of the procedure and the instructions executed when the procedure terminates
uint16 PROGRAM_PARSE=11
# Check the contents of the TPD file
uint16 TPD_CONTENT=12
# Check the contents of the TPD file
uint16 TPD_POINT_COUNT=13
# Check whether the web interface is faulty or whether the status feedback is faulty
uint16 INSTRUCTION_EXECUTION=14
# Re-record
uint16 TPD_RECORD_LIMIT=15
# Do not reload the protocol repeatedly
uint16 PROTOCOL_ALREADY_LOADED=16
# Please load the protocol first
uint16 PROTOCOL_NOT_LOADED=17
# Stop the program before proceeding with other operations
uint16 PROGRAM_RUNNING=18
# Check the connection between the robot and the sensor network and the loading of the sensor protocol
uint16 POSTURE_SENSOR_COMM=19
# Swing welding requires the tool coordinate system to be set with a tool number other than 0
uint16 WELD_TOOL_UNSET=20
# Please remove the active extension shaft first
uint16 EXTERNAL_AXIS_ACTIVE=21
# Please set the tool coordinate system first
uint16 THREE_POINT_TOOL_UNSET=22
# Check the connection between the robot and the sensor network and the loading of the sensor protocol
uint16 POSITION_SENSOR_ACQUISITION=23
# Please reduce the value of the first four points of attitude change
uint16 EIGHT_POINT_POSTURE_CHANGE=24
# Redeploy or re-identify
uint16 CALCULATION_FAILED=25
# Please switch to the base coordinate system (tool coordinate system with tool number 0\) first
uint16 EIGHT_POINT_BASE_FRAME=26
# Please re-calibrate
uint16 EIGHT_POINT_CALCULATION=27
# Check whether the position is reasonable
uint16 INVERSE_KINEMATICS=28
# Check that the joint data is within a reasonable range
uint16 SERVO_JOINT_LIMIT=29
# Please disconnect the power and restart the control box
uint16 RESET_FAILED=30
# Please disconnect power and restart the control box
uint16 EMERGENCY_STOP_RESET=31
# Switch to drag mode and move the joint into the soft limit range
uint16 JOINT_LIMIT=32
# Please check the external shaft setting
uint16 EXTERNAL_AXIS_ZERO_CONFIG=33
# Please check whether the workpiece number is reasonable
uint16 WORKPIECE_NUMBER=34
# Please switch to the workpiece coordinate system of workpiece number 0
uint16 WORKPIECE_ZERO_REQUIRED=35
# Please reduce the length of the file name
uint16 FILE_NAME_TOO_LONG=36
# Please check whether the tool number is reasonable
uint16 TOOL_NUMBER=37
# Please change your posture
uint16 POSTURE_INVALID=38
# Please check the Socket name
uint16 SOCKET_NAME=39
# Check whether the speed percentage is reasonable
uint16 SPEED_PERCENT_LIMIT=40
# Please check the external shaft setting
uint16 EXTERNAL_AXIS_NOT_ZERO=41
# Insert intermediate postures for transition
uint16 POSTURE_CHANGE_TOO_LARGE=42
# Please configure the conveyor belt DI detection port
uint16 CONVEYOR_DI_UNCONFIGURED=43
# Please check the target posture setting
uint16 ROBOT_POSTURE_LIMIT=44
# Please check the external shaft setting
uint16 EXTERNAL_AXIS_NOT_ACTIVE=45
# Please check the external shaft setting
uint16 EXTERNAL_AXIS_CALIBRATION_REQUIRED=46
# Please check the external shaft setting
uint16 EXTERNAL_DRIVER_CONFIG=47
# Please check the external shaft setting
uint16 EXTERNAL_DRIVER_CONFIG_TIMEOUT=48
# Please check the status of the external shaft
uint16 EXTERNAL_AXIS_ENABLE=49
# Please check the status of the external shaft
uint16 EXTERNAL_DRIVER_INFO=50
# Please check the status of the external shaft
uint16 EXTERNAL_DRIVER_INFO_TIMEOUT=51
# Please check the external shaft setting
uint16 SYNC_SINGLE_STEP=52
# Stimulate the activity sensor
uint16 FORCE_SENSOR_NOT_ACTIVE=59
# Switch the force sensor coordinate system to the tool
uint16 FORCE_SENSOR_FRAME=60
# Please set the zero point of force/torque sensor first
uint16 FORCE_SENSOR_NOT_ZEROED=61
# Please first reset the load of the force/torque sensor
uint16 FORCE_SENSOR_LOAD_NOT_ZEROED=62
# Contact the after-sales engineer to view the controller log
uint16 SYSTEM_TIME=63
# Contact the after-sales engineer to view the controller log
uint16 INSTRUCTION_QUEUE_ADD=64
# Check whether the data at midpoint 1 is correct
uint16 CIRCLE_MIDPOINT_1=66
# Check whether the data at midpoint 2 is correct
uint16 CIRCLE_MIDPOINT_2=67
# Check whether the data at the midpoint 3 is correct
uint16 CIRCLE_MIDPOINT_3=68
# Check that the midpoint data is correct
uint16 ARC_MIDPOINT=69
# Check whether the target point data is correct
uint16 ARC_TARGET=70
# Check whether the communication status of the clamp is normal
uint16 CLAW_MOVEMENT=73
# Check whether the checkpoint data is correct
uint16 LINEAR_POINT=74
# Check that the IO number is within the range
uint16 CHANNEL=75
# Check whether the IO signal is input or whether the wiring is correct
uint16 WAIT_TIMEOUT=76
# Re-record the teaching path
uint16 TPD_POINT=82
# Change to the tool coordinate system used for TPD teaching
uint16 TPD_TOOL_MISMATCH=83
# Please check the laser sensor parameter configuration to ensure that the sensor can identify the weld seam
uint16 WELD_SEAM_LOCATION=84
# Please check whether the position parameters of the instruction point, tool number and workpiece number are configured
uint16 LINE_INSTRUCTION=85
# Contact the after-sales engineer to check the controller log
uint16 EXTERNAL_CONFIG_FILE=90
# Contact the after-sales engineer to view the controller log
uint16 PERIPHERAL_CONFIG_VERSION=91
# Contact the after-sales engineer to view the controller log
uint16 PERIPHERAL_CONFIG_READ=92
# Please reduce the number of spline instruction points
uint16 SPLINE_POINT_COUNT=93
# Check whether the checkpoint data is correct
uint16 SPLINE_POINT=94
# Check whether the spline parameters are reasonable
uint16 SPLINE_PARAMETERS=95
# Please check the welding wire location parameter configuration
uint16 WIRE_POSITIONING=96
# Please line up for data recording
uint16 EMPTY_RECORDED_DATA=97
# Please configure the main program first
uint16 MAIN_PROGRAM_UNCONFIGURED=98
# Please check the status of the safety stop signal
uint16 SAFETY_STOP=99
# Please configure the origin of the job first
uint16 JOB_ORIGIN_UNCONFIGURED=100
# Please enable the robot first
uint16 ROBOT_NOT_ENABLED=101
# Enable the external shaft first
uint16 EXTERNAL_AXIS_NOT_ENABLED=106
# Check that the starting point data is correct
uint16 HELIX_START_POINT=108
# Please close the drag configuration to enter the interference area
uint16 DRAG_IMPEDANCE_INTERFERENCE=110
# Please turn off the drag teaching lock freedom function
uint16 DRAG_TEACHING_LOCK=111
# Check whether the target position is reasonable
uint16 POSITIONING_POSTURE_UNREACHABLE=112
# Please check the value of the DMP instruction position
uint16 DMP_POINT=114
# Please check the target point information of circular motion
uint16 CIRCLE_POINT=115
# Please load the extension shaft communication first
uint16 EXT_AXIS_DRIVER_UNLOADED=116
# Please check the load weight parameter value under the force sensor
uint16 FORCE_SENSOR_LOAD_WEIGHT=118
# Please check the position parameter value of the load center of mass under the force sensor
uint16 FORCE_SENSOR_LOAD_COG=119
# Please check the trajectory command point
uint16 TRAJECTORY_POINT=120
# Please check the trajectory command point
uint16 TRAJECTORY_POINT_COUNT=121
# Please check whether the current robot load is too large
uint16 JOINT_TORQUE_LIMIT=122
# Please exit the drag mode
uint16 DRAG_MODE_ACTIVE=123
# Please configure the extended axis parameters first
uint16 EXT_AXIS_PARAMETERS=124
# Contact the after-sales engineer to view the controller log
uint16 WELD_RECOVERY_PROGRAM=125
# Contact the after-sales engineer to view the controller log
uint16 WELD_RECOVERY_ARC_POINT=126
# Contact the after-sales engineer to view the controller log
uint16 WELD_RECOVERY_GENERATE=127
# Please switch to manual mode
uint16 MANUAL_MODE_REQUIRED=128
# Please switch to the new dynamics mode first
uint16 NEW_DYNAMICS_MODE=129
# Check whether the name of the checkpoint is correct
uint16 POINT_TABLE_MISSING=130
# Please switch to drag mode / force sensor assisted drag mode first
uint16 DRAG_MODE_REQUIRED=133
# Check the load of the force/torque sensor
uint16 FORCE_SENSOR_UNLOADED=134
# Contact the after-sales engineer to view the controller log
uint16 FORCE_SENSOR_MATRIX=135
# Please first configure the DI port of the detection switch
uint16 DETECTION_DI_UNCONFIGURED=138
# Please apply the extended axis coordinate system with a number other than 0 first
uint16 EXT_AXIS_FRAME_REQUIRED=139
# Please activate the current extension axis first
uint16 EXT_AXIS_ACTIVATION_REQUIRED=140
# Please check that the teaching point information exists
uint16 TEACHING_POINT_MISSING=143
# Please check whether the LUA file exists
uint16 LUA_FILE_MISSING=144
# Please check whether the joint configuration of the target position and the current position has changed
uint16 JOINT_CONFIGURATION_CHANGED=151
# Check whether the distance between the welding instruction point and the current point is too small
uint16 WELD_POINT_SPACING=152
# Please check the spacing of arc instruction points
uint16 ARC_POINT_SPACING=153
# Please check the target point information of the instruction
uint16 JOINT_POINT=154
# Please check the content of the background program
uint16 BACKGROUND_PROGRAM_START=155
# Contact the after-sales engineer to view the controller log
uint16 BACKGROUND_PROGRAM_PAUSE=156
# Contact the after-sales engineer to check the controller log
uint16 BACKGROUND_PROGRAM_RESUME=157
# Contact the after-sales engineer to view the controller log
uint16 BACKGROUND_PROGRAM_DELETE=158
# Please check the number of background programs running
uint16 BACKGROUND_PROGRAM_LIMIT=159
# Contact the after-sales engineer to view the controller log
uint16 SPEED_PROFILE_VALIDATION=160
# Contact the after-sales engineer to check the controller log
uint16 LOAD_PROFILE_VALIDATION=161
# Please adjust the extension shaft movement to synchronous movement
uint16 LASER_TRACKING_ASYNC_AXIS=162
# Multiple while nesting cannot be restored from welding interruption
uint16 WELD_MULTIPLE_WHILE=163
# Please set the segment welding information
uint16 SEGMENT_WELD_INFO=164
# Check CNC communication
uint16 CNC_COMMUNICATION=165
# CNC timeout
uint16 CNC_TIMEOUT=166
# Check CNC communication
uint16 CNC_CONNECTION_TIMEOUT=167
# Check the number of monitored numeric variables
uint16 MONITORED_NUMERIC_LIMIT=168
# Check the number of monitored character variables
uint16 MONITORED_CHARACTER_LIMIT=169
# Please check the workpiece coordinate system
uint16 WORKPIECE_FRAME_NOT_APPLIED=170
# Please check the joint soft limits
uint16 JOINTS_OUTSIDE_SOFT_LIMIT=171
# Motion speed cannot be 0
uint16 ZERO_MOTION_SPEED=172
# Acceleration smoothing must be enabled for extended axis Blend motion
uint16 EXT_AXIS_BLEND_SMOOTHING=173
# Current robot model does not support this function
uint16 UNSUPPORTED_ROBOT_FUNCTION=174
# Please configure socket connection information
uint16 SOCKET_UNCONFIGURED=175
# Socket communication is connected
uint16 SOCKET_CONNECTED=176
# Please check the socket connection
uint16 SOCKET_NOT_CONNECTED=177
# Firmware version is too high, current software version does not support it
uint16 FIRMWARE_VERSION_TOO_HIGH=178
# Firmware version is too low, current software version does not support it
uint16 FIRMWARE_VERSION_TOO_LOW=179
# Contact after-sales engineer to check controller logs
uint16 GCODE_FILE_OPEN=180
# Contact after-sales engineer to check controller logs
uint16 GCODE_PARSE=181
# Contact after-sales engineer to check controller logs
uint16 PHYSICAL_SPEED_LIMIT=182
# Contact after-sales engineer to check controller logs
uint16 PHYSICAL_ACCELERATION_LIMIT=183
# Contact after-sales engineer to check controller logs
uint16 EXTERNAL_AXIS_ENABLE_FAILED=184
# Contact after-sales engineer to check controller logs
uint16 MOTION_ERROR_STOP=185
# Contact after-sales engineer to check controller logs
uint16 MOTION_EMERGENCY_STOP=186
# Contact after-sales engineer to check controller logs
uint16 COLLISION_LEVEL=187
# Force control and impedance control are both enabled
uint16 FORCE_IMPEDANCE_CONFLICT=188
# Contact after-sales engineer to check controller logs
uint16 KERNEL_UPGRADE_VERSION=189
# Contact after-sales engineer to check controller logs
uint16 KERNEL_UPGRADE=190
# Linear rack guide is not activated
uint16 LINEAR_RACK_NOT_ACTIVE=191
# Current linear rack guide collision detection is only for upright installation
uint16 LINEAR_RACK_INSTALLATION=192
# Contact after-sales engineer to check controller logs
uint16 TORQUE_CONTROL_SPEED_LIMIT=193
# Contact after-sales engineer to check controller logs
uint16 TORQUE_CONTROL_POWER_LIMIT=194
# FR30L base tilt angle must be 0
uint16 FR30L_BASE_TILT=195
# The photoelectric sensor signal has not been triggered.
uint16 PHOTOELECTRIC_SENSOR_NOT_TRIGGERED=196
# Calibrate the joint torque sensor again
uint16 JOINT_TORQUE_SENSOR_CALIBRATION=197
# Contact after-sales engineer to check controller logs
uint16 JOINT_TORQUE_SENSOR_ANOMALY=198
# The multi-cube can only be configured for interference within the specified range.
uint16 MULTI_INTERFERENCE_RANGE=199
# The robot is operating in intermittent mode.
uint16 INTERMITTENT_MODE=200
# Contact after-sales engineer to check controller logs
uint16 SWING_CONFIG_FILE=201
# Contact after-sales engineer to check controller logs
uint16 INTERFERENCE_CONFIG_FILE=202
# Contact after-sales engineer to check controller logs
uint16 END_485_COMMUNICATION=203
# Check the instruction sending frequency
uint16 INSTRUCTION_QUEUE_FULL=204
# Close the endpoint protocol first
uint16 ENDPOINT_PROTOCOL_CONFIGURED=205
# First, clear the currently adapted device
uint16 ADAPTED_DEVICES_CONFIGURED=206
# Automatic mode safe stop requires three-position enablement with the teaching device.
uint16 AUTO_SAFE_STOP_ENABLE=207
# The current DI configuration is occupied and cannot be changed.
uint16 DI_CONFIGURATION_OCCUPIED=208
# Not in tracking mode
uint16 TRACKING_MODE_REQUIRED=209
# Calibrate the conveyor belt again.
uint16 CONVEYOR_CALIBRATION=210
