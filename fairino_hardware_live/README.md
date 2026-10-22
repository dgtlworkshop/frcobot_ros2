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
ros2 control switch_controllers --deactivate fairino_controller --strict
```

2. Attempt to trigger recovery

```sh
ros2 service call /fairino/recover std_srvs/srv/Trigger '{}'
```

This should be successful

```py
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=True, message='Recovered; activate the controller and submit a NEW trajectory')
```

If the response says to disable the controller, you can continue as the controller is already active.

```sh
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=False, message='Deactivate the trajectory controller and wait for fresh connected feedback')
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
