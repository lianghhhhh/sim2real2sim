# kros_car

ROS 2 Jazzy package for the KROS Car — a 4WD skid-steer mobile robot. The arm and gripper on the CAD model are not driven: their joints are fixed in the URDF. Uses `ros2_control` with direct serial I/O in a single C++ hardware plugin.

## Architecture

```
/base_controller/cmd_vel      /joint_command
        |                           |
  base_controller      wheel_velocity_controller
  (diff_drive)         (per-wheel speeds)
        \___________ one active ____/
                     |
   KrosCarSystem (C++ hardware plugin, 50 Hz)
                     |
   ESP32 wheel controller, JSON over USB serial
                     |
   4 x DDSM115, RS-485
```

The hardware plugin talks to the serial device directly via **libserial**. A background **wheel IO thread** sends the four wheel targets to the ESP32 at 50 Hz and reads the feedback it reports at 50 Hz.

The wheel motors run in their built-in **speed loop**. The ESP32 firmware (`wildbot_wheelctrl`) owns the RS-485 bus timing; see its README for the wire protocol.

## Hardware

| Component | Hardware | Protocol | Default Port |
|---|---|---|---|
| Wheels (x4) | DDSM115 hub motors behind an ESP32 | JSON over USB serial, 460800 baud | `/dev/usb_wheel` |

- Wheel radius: 50.35 mm, max RPM: 100

## Topics

| Topic | Type | Dir | Description |
|---|---|---|---|
| `/base_controller/cmd_vel` | `geometry_msgs/msg/TwistStamped` | sub | Drive command (linear.x + angular.z), when `base_controller` is active |
| `/joint_command` | `sensor_msgs/msg/JointState` | sub | Per-wheel speed command, when `wheel_velocity_controller` is active. Wheel speed [rad/s] in `velocity`, looked up by joint name |
| `/base_controller/odom` | `nav_msgs/msg/Odometry` | pub | Wheel odometry from the measured wheel positions; no `odom -> base_link` TF is published by the controller |
| `/joint_states` | `sensor_msgs/msg/JointState` | pub | All joints at the controller rate. Wheels: measured position, velocity and effort |
| `/wheel_feedback` | `sensor_msgs/msg/JointState` | pub | Wheel position, velocity and effort, one message per feedback line from the ESP32 (50 Hz), stamped with the time the ESP32 sent it |

### Wheel feedback

| Quantity | Source | Unit |
|---|---|---|
| position | motor position, unwrapped across turns, zero at activation | rad |
| velocity | motor speed (reported in whole rpm, so the resolution is 0.105 rad/s) | rad/s |
| effort | motor torque current x `wheel_torque_constant` | N·m |

All three use the joint's sign: positive is forward on every wheel. Divide effort by `wheel_torque_constant` to get the motor current in A.

Use `/wheel_feedback` for logging. `/joint_states` carries the same values but is stamped by the controller loop, so its timestamps are quantized to 20 ms.

## Joints

| Joint | Motor ID | Command | State |
|---|---|---|---|
| `front_left_joint` | 1 | velocity | position, velocity, effort |
| `rear_left_joint` | 2 | velocity | position, velocity, effort |
| `front_right_joint` | 3 | velocity | position, velocity, effort |
| `rear_right_joint` | 4 | velocity | position, velocity, effort |

## Dependencies

- ROS 2 Jazzy
- [`kros_car_description`](../kros_car_description) — URDF/xacro with `ros2_control` hardware block
- `libserial-dev` (`apt install libserial-dev`)
- `joint_state_publisher` / `joint_state_publisher_gui` are **not** required for bringup

## Build

```bash
cd /path/to/ros2_ws
colcon build --packages-select kros_car_description kros_car
source install/setup.bash
```

Unit tests for the wheel protocol (no hardware needed):

```bash
colcon test --packages-select kros_car && colcon test-result --verbose
```

## Launch

```bash
ros2 launch kros_car bringup.launch.py
```

### Launch Arguments

| Argument | Default | Description |
|---|---|---|
| `enable_wheel` | `true` | Load the wheel controllers |
| `wheel_controller` | `base_controller` | Wheel controller that starts active: `base_controller` or `wheel_velocity_controller` |
| `model` | `kros_car_description/.../kros_car.xacro` | Robot URDF xacro path |
| `controllers_file` | `/configs/controllers.yaml` if present, otherwise package share `config/controllers.yaml` | Controller config path |
| `...` | | Every hardware parameter below is also a launch argument |

Hardware parameters are defined in the URDF `ros2_control` block (`kros_car_description/urdf/kros_car.xacro`) and exposed through `bringup.launch.py` as xacro-backed launch arguments.

Config precedence is:

1. explicit launch argument
2. `/configs/*.yaml`
3. defaults in `bringup.launch.py` (the same values as `config/hardware.yaml`)

Override controller and hardware settings inside a container without touching the installed package:

```bash
docker run -v ./configs:/configs ...
```

### Hardware Parameters

| Parameter | Default | Description |
|---|---|---|
| `wheel_port` | `/dev/usb_wheel` | Wheel serial port |
| `wheel_baudrate` | `460800` | Wheel serial baud rate; must match `SERIAL_BAUDRATE` in the firmware |
| `wheel_motor_ids` | `1,2,3,4` | Wheel motor IDs (CSV), unique and within 1-4 |
| `wheel_motor_directions` | `1,1,-1,-1` | Motor direction multipliers (CSV) |
| `max_rpm` | `100` | Max wheel RPM |
| `acceleration_act` | `1` | DDSM115 speed-loop acceleration time, 0.1 ms per rpm |
| `wheel_command_rate_hz` | `50` | Rate the wheel targets are sent to the ESP32 |
| `wheel_heartbeat_ms` | `500` | The ESP32 stops the wheels when no command arrives for this long (-1 = off) |
| `wheel_feedback_timeout_ms` | `200` | A wheel without feedback for this long is reported as stopped |
| `wheel_torque_constant` | `0.75` | N·m per A, turns motor current into joint effort. Datasheet value |
| `wheel_require_feedback` | `true` | Stop the wheels while any wheel has no fresh feedback or is not in speed loop mode |
| `wheel_joints` | `front_left_joint,...` | Wheel joint names (CSV), in `wheel_motor_ids` order |

## Usage

Drive the base with a twist (`base_controller` active):

```bash
ros2 topic pub /base_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.2}, angular: {z: 0.4}}}"
```

Drive the base with wheel speeds (`wheel_velocity_controller` active). The message has to keep coming: the wheels stop 0.3 s after the last one.

```bash
ros2 topic pub -r 50 /joint_command sensor_msgs/msg/JointState \
  "{name: [front_left_joint, rear_left_joint, front_right_joint, rear_right_joint], velocity: [2.0, 2.0, 2.0, 2.0]}"
```

Switch between the two wheel controllers at runtime:

```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController \
  "{activate_controllers: [wheel_velocity_controller], deactivate_controllers: [base_controller], strictness: 2}"
```

Introspect:

```bash
ros2 topic echo /joint_states
ros2 topic echo /wheel_feedback
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers
```

## Controllers

Defined in `config/controllers.yaml` (update rate: 50 Hz):

- **base_controller** (`diff_drive_controller`) — left: `front_left_joint`, `rear_left_joint`; right: `front_right_joint`, `rear_right_joint`. Applies velocity and acceleration limits and the `wheel_separation_multiplier` slip correction.
- **wheel_velocity_controller** (`kros_car/WheelVelocityController`) — forwards per-wheel speeds from `/joint_command` with no limits or corrections in between, and commands a stop when no message arrives within `command_timeout` (0.3 s). A wheel a message does not name is commanded to stop; a message without `velocity` is ignored.
- **joint_state_broadcaster** — publishes combined `/joint_states`.

`base_controller` and `wheel_velocity_controller` command the same wheels, so only one is active at a time.

`base_controller` is configured with `enable_odom_tf: false`, so it publishes `/base_controller/odom` but does not own the `odom -> base_link` transform.

## Safety behaviour

| Situation | What happens |
|---|---|
| Sender of `/joint_command` stops or crashes | `wheel_velocity_controller` commands zero after 0.3 s |
| Sender of `cmd_vel` stops | `base_controller` commands zero after `cmd_vel_timeout` |
| This node freezes or the USB link drops | The ESP32 stops the wheels after `wheel_heartbeat_ms` |
| A motor stops answering for `wheel_feedback_timeout_ms` | All wheels are commanded to stop until it answers again |
| A motor is not in speed loop mode | All wheels are commanded to stop and that motor is switched back once it is below 10 rpm |
| A motor reports an error code | Logged with the decoded bits |

## Threading Model

```
Controller Manager thread (50 Hz)
  +-- read()  — copies the latest wheel feedback into the state interfaces
  +-- write() — updates latest_wheel_rpms_ (wheel_cmd_mutex_)

Wheel IO thread
  +-- every 20 ms: sends the four targets in one line; every 2 s: repeats the heartbeat setting
  +-- reads feedback lines, updates wheel_feedback_ (wheel_fb_mutex_), publishes /wheel_feedback
```

## Package Structure

```
kros_car/
  config/hardware.yaml                # Hardware defaults and /configs template
  config/controllers.yaml             # Controller configuration
  include/kros_car/
    kros_car_system.hpp               # Hardware plugin header
    wheel_protocol.hpp                # Wheel wire protocol and unit conversions (no ROS)
    wheel_velocity_controller.hpp     # Per-wheel speed controller
  src/
    kros_car_system_common.cpp        # CSV parsing + serial baud helpers
    kros_car_system_threads.cpp       # Wheel IO thread loop
    kros_car_system_lifecycle.cpp     # on_init / on_activate / on_deactivate
    kros_car_system_io.cpp            # read() / write() hardware interface hooks
    wheel_protocol.cpp
    wheel_velocity_controller.cpp
  test/
    test_wheel_protocol.cpp           # Unit tests for wheel_protocol
  launch/
    bringup.launch.py                 # Main launch file
    car.launch.py                     # Wrapper (includes bringup)
  kros_car_hardware_plugins.xml       # pluginlib descriptor, hardware plugin
  kros_car_controller_plugins.xml     # pluginlib descriptor, controller
  CMakeLists.txt
  package.xml
```

## Notes

- Uses the Jazzy-native `ros2_control` API (framework-managed interfaces, `set_state`/`get_command`).
- The diff-drive controller does not publish `odom -> base_link` TF by default; external localization is expected to own that transform.
- The `write()` method does not do serial I/O directly; it updates a shared command buffer that the wheel IO thread consumes.
- The sign of the motor's position and torque-current feedback is assumed to follow the sign of its speed. This has not been checked on the car yet.
