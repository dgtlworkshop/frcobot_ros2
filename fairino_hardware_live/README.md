# Fairino Hardware (v3.9.9)

This is large rewrite of the Fairino ros2_control drivers to handle robot control authority changes.

## Usage

### Boot Sequence

1. Start your MoveIt Pipeline (TODO: Example)
2. Attempt a control recovery

```sh
ros2 service call /fairino/recover std_srvs/srv/Trigger '{}'
```

3. Activate the FairinoSystem

```sh
ros2 control set_hardware_component_state FairinoSystem active
```

4. Activate `fairino_controller`

```sh
ros2 control switch_controllers --activate fairino_controller --strict
```

### Recovering from a Fault

1. Deactivate controller

```sh
ros2 control switch_controllers --deactivate fairino_controller
```

> [!NOTE]
> `--strict` is not used here as this command could result in no controllers active

2. Attempt to trigger SDK recovery

```sh
ros2 service call /fairino/recover std_srvs/srv/Trigger '{}'
```

This should be successful

```py
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=True, message='Recovered; activate the controller and submit a NEW trajectory')
```

If the response says the servo control is still claimed, deactivate the controller first.
A failed response names the blocking condition:

| Message | Action |
|---|---|
| `Servo control still claimed; ...` | Deactivate `fairino_controller` |
| `Robot stop pending until the SDK reconnects; ...` | Restore the network and retry |
| `Waiting for fresh connected robot feedback` | Restore the network and retry |
| `Robot fault ...` / `... alarm: abnomalStop=1 ...` | Resolve the cause, then `ResetAllError()` (below) |
| `Robot not ready: rbtEnableState=0 ...` | `RobotEnable(1)` (below) |

After a network loss the robot controller typically reports `abnomalStop=1` and
disables itself. Recovery never clears robot alarms; do so explicitly:

```sh
ros2 service call /fairino_remote_command_service fairino_msgs/srv/RemoteCmdInterface "{cmd_str: 'ResetAllError()'}"
ros2 service call /fairino_remote_command_service fairino_msgs/srv/RemoteCmdInterface "{cmd_str: 'RobotEnable(1)'}"
ros2 service call /fairino/recover std_srvs/srv/Trigger '{}'
```

If the hardware component was deactivated, reactivate it before the controller:

```sh
ros2 control set_hardware_component_state FairinoSystem active
```

3. Activate controller

```sh
ros2 control switch_controllers --activate fairino_controller --strict
```

## Development

### Upgrading to newer upstream fairino_hardware versions

To upgrade to a new version, only a few files typically change:

- These C++ files can be migrated over with no changes
  - [`src/command_server.cpp`](./src/command_server.cpp)
  - [`src/CNDE_thread.cpp](./src/CNDE_thread.cpp)
- These header files can be migrated over with no changes
  - [`include/fairino_hardware/command_server.hpp`](./include/fairino_hardware/command_server.hpp)
  - [`include/fairino_hardware/CNDE_thread.hpp`](./include/fairino_hardware/CNDE_thread.hpp)
  - [`include/fairino_hardware/data_type_def.h`](./include/fairino_hardware/data_type_def.h)
- [`libfairino`](../libfairino) will need be upgraded
  1.  Switch the branch of the [.gitmodules](../.gitmodules)
  2.  Switch the SDK to the same branch [fairino-cpp-sdk](../libfairino/fairino-cpp-sdk)
  3.  Update the [package.xml](../libfairino/package.xml) to the same version
- Update [`package.xml`](./package.xml) versions and [`CMakeLists.txt`](./CMakeLists.txt) as needed
