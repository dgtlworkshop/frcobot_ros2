# Fairino 3.9.9 hardware (ROS 2 Humble)

This package uses one SDK owner for ros2_control, robot feedback and the legacy
`fairino_remote_command_service`. Do **not** also launch `ros2_cmd_server` or the
old CNDE node. The standalone command server still works without ros2_control;
an advisory per-user process lock rejects a second SDK owner from this package.
Other SDK programs and other Unix users must be excluded operationally.
The old auxiliary CNDE UDP/shared-memory feed is no longer started; consumers
should use the SDK-backed `nonrt_state_data` topic instead.

The separation is: standard JTC interpolation -> position interfaces
-> hardware command mailbox -> SDK worker -> `ServoJ`. The
`fairino_hardware/FaultAwareJointTrajectoryController` extends Humble's JTC only
to consume `fairino/motion_allowed`, abort actions and inhibit further sampling.
It does not implement speed scaling: Fairino's manual/auto speed percentages
are published but are not established as equivalent execution-time scaling.

## Real hardware startup

Build `libfairino`, `fairino_msgs` and this package, and source the install space.
The SDK checkout must match firmware 3.9.9 (currently submodule tag
`v2.3.9-robot3.9.9`). Install the Humble ros2_control, ros2_controllers and xacro
dependencies using the workspace's usual rosdep procedure.

```sh
ros2 launch fairino_hardware_live real_hardware.launch.py robot_ip:=192.168.58.2 model:=fairino20_v6
```

This configures the hardware and starts monitoring, but deliberately leaves the
hardware and trajectory controller **inactive**. Verify robot mode, alarms,
safety equipment, clear workspace and joint order before enabling motion.
SDK connection/configuration errors leave this monitor-only configuration
running with error diagnostics; hardware activation is refused. After an initial
setup failure, cleanup/configure the hardware again rather than acknowledging
unverified feedback configuration through recovery.
Enable the robot in automatic mode using the vendor's normal procedures, with
no LUA program executing. Then:

```sh
ros2 service call /fairino/recover std_srvs/srv/Trigger '{}'
ros2 control set_hardware_component_state FairinoSystem active
ros2 control switch_controllers --activate fairino_controller --strict
```

Submit new goals to `/fairino_controller/follow_joint_trajectory`. Adapt MoveIt's
controller mapping to that name. Existing MoveIt demo configurations remain mock
hardware; this launch does not start MoveIt or a second controller manager.
Alternatively include `urdf/fairino.ros2_control.xacro` in your complete robot
description and use `config/controllers.yaml`. Do not include a second
ros2_control block for the same joints. SDK axes 1..6 must match the six declared
joints **in order**. State interfaces support position (required), velocity and
effort; commands are position only. ROS uses radians, radians/s and Nm; SDK
feedback topics retain vendor units (degrees, mm, Nm).

## Fault and recovery policy

`programRunState=200` is accepted as an explicit compatibility exception despite
being undocumented in this SDK. Entering this value logs a warning; otherwise
ready hardware reports diagnostic `WARN` while it persists. This is not evidence
that no LUA program is running: verify that independently before motion.
Only `0` and this specific exception pass the program-state gate; all other
readiness, fault-latch and explicit-recovery requirements remain unchanged.

The independent 8 ms worker reads `GetSDKComState` and `GetRobotRealTimeState`
continuously from configuration through inactive, active and hardware error
states. It publishes through a separate ROS executor, not through JTC:

| Surface | Meaning |
|---|---|
| `nonrt_state_data` (`fairino_msgs/RobotNonrtState`) | Existing full vendor state mapping, reliable/transient-local, including alarms and error codes |
| `fairino/diagnostics` (`diagnostic_msgs/DiagnosticArray`) | Connected/fresh, feedback age, latched reason, control claim, enable/robot state and safety stops |
| `fairino/{motion_allowed,connected,fault_latched,feedback_age,main_error_code,sub_error_code}` | ros2_control state interfaces, with configurable `state_prefix` |
| `fairino/recover` (`std_srvs/Trigger`) | Explicitly acknowledge a resolved fault; never resets robot errors or enables motion |

Disconnected/stale messages retain the last measured values and original
`timestamp` (legacy Unix seconds); `reconnect_flag=1` marks them invalid. Never
interpret a repeated timestamp or a default all-zero startup message as current
feedback. Use diagnostics for subsecond freshness and communication state.
Last valid data is transient-local for late subscribers. Publication ends only
on cleanup/shutdown/destruction, not on controller or hardware deactivation.

SDK errors, frozen/missing robot timestamps, robot main/sub errors, emergency and
safety stops, collision/controller/drive alarms, disabled/manual/drag/paused
states, competing LUA execution, non-finite joint state/commands and command
watchdog expiry inhibit writes and latch a fault. Automatic reconnection can
restore monitoring, **not motion**. A fault requests one best-effort
`StopMotion`/`ServoMoveEnd`; failed stop calls are explicitly logged. Commands are
discarded and no further ServoJ is sent. The guarded JTC aborts the active action
with `PATH_TOLERANCE_VIOLATED` and a hardware-fault explanation and returns
controller `ERROR`. Because Humble only logs that return value, a non-realtime
controller timer explicitly requests strict deactivation through
`/controller_manager/switch_controller` (configurable `controller_manager_name`).
If that service is unavailable, goals remain aborted/rejected and writes remain
inhibited. No controller is automatically reactivated.

Hardware `read`/`write` intentionally stay `OK` for monitored robot faults:
the exported permission becomes zero, the worker refuses commands, and the
guarded JTC escalates the controller error. Returning hardware `ERROR` can make
Humble stop updating those interfaces before the action sees the fault. **Use
the guarded JTC**, not the unguarded upstream JTC, for this error-result contract.
The hardware write gate still protects against commands from other controllers.

Recovery requires controller deactivation/released interfaces, fresh connected
feedback and a ready robot. Resolve the cause using vendor procedures first:

```sh
ros2 control switch_controllers --deactivate fairino_controller --strict
ros2 service call /fairino/recover std_srvs/srv/Trigger '{}'
ros2 control switch_controllers --activate fairino_controller --strict
```

Servo release and claim never wait for the SDK on the controller-manager thread.
While the SDK is disconnected (or busy), the `StopMotion`/`ServoMoveEnd` request is
deferred (`stop_pending=1` in diagnostics) and sent by the worker once the SDK
reconnects; recovery and controller activation are refused until it has run.
Diagnostics `readiness` names the currently blocking condition and alarm flags,
while the status message keeps the first latched fault. After a network loss the
robot is typically disabled and alarmed: clear it with `ResetAllError` and
`RobotEnable` (allowed through the command service while unclaimed) before
calling recover.

If the hardware itself was deactivated, activate it before the controller.
Activation seeds commands from measured positions; an interrupted trajectory
must be discarded by the client and a **new** goal submitted. Failed recovery
returns `success=false` with the unresolved condition.
Servo entry requires a stopped robot. External-axis targets are held at their
measured positions on servo entry, not implicitly commanded to zero; external
axis trajectory control is not provided.

While a controller claims motion, legacy SDK service commands are refused
(`cmd_res=-1`, with a log); use topics for feedback. `StopMotion` is always
allowed and latches an interruption. With ros2_control configured but unclaimed,
the shared command service allows only `GetSDKVersion`, `GetErrorCode`,
`GetControllerIP`, `GetRobotVersion`, `ResetAllError`, `RobotEnable`, `Mode`,
`SetDO`, `SetToolDO`, `SetAO` and `SetToolAO`. This deliberately prevents legacy
motion/configuration commands from bypassing ros2_control. Connection/feedback
reconfiguration is owned by the session. Busy SDK calls fail explicitly rather
than queueing. Standalone mode retains the legacy command set except connection/
feedback reconfiguration; it is not a fault-guarded trajectory executor.

## Timing and commissioning limits

The worker configures feedback at 8 ms and sends single-point UDP ServoJ
(`comType=1`, `cmdT=0.008`); the example manager rate is 125 Hz.
`feedback_timeout_ms` and `command_timeout_ms` default to 100 ms and accept
16..1000 ms. Tune only after measuring timing and establishing stopping-distance
requirements. Lifecycle servo entry/exit uses XML-RPC and may block. Cyclic
read/write only copy bounded mailboxes; SDK calls run outside controller-manager.
No target queue is replayed after a delay.

This is **not safety-rated**, hard-real-time, or a replacement for the robot's
safety controller. UDP send success does not acknowledge physical execution.
There is no verified robot-side receive watchdog in this integration; establish
one and the vendor's stop-on-network-loss behavior before commissioning.
Host watchdogs cannot stop a robot across a severed connection or a stalled SDK.
The supplied SDK copies its asynchronously updated state without a documented
atomic snapshot guarantee; telemetry must not be used as a safety-rated signal.
Validate firmware/SDK field availability, especially `robotTime`,
`rbtEnableState`, safety flags and UDP support, on the actual controller.

Before enabling a real arm, exercise faults/disconnects, frozen feedback,
watchdog timeouts, servo failures, cancellation, tolerance errors, startup hold
and explicit recovery in simulation/mocks, then commission under vendor
procedures with independent safety equipment and reduced risk.
