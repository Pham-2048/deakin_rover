# Deakin Rover — ROS 2 Architecture Document

> **ROS 2 Distribution:** Jazzy (Ubuntu 24.04)
> **Compute:** Raspberry Pi 5 / NVIDIA Jetson Orin Nano
> **Last updated:** 2026-02-13

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Package Map](#2-package-map)
3. [Node Specifications](#3-node-specifications)
4. [Complete Topic Map](#4-complete-topic-map)
5. [Complete Service Map](#5-complete-service-map)
6. [Data Flow Diagrams](#6-data-flow-diagrams)
7. [TF Tree](#7-tf-tree)
8. [Launch File Structure](#8-launch-file-structure)
9. [Implementation Phases](#9-implementation-phases)
10. [4-Wheel Steering Upgrade Path](#10-4-wheel-steering-upgrade-path)
11. [Dependencies](#11-dependencies)
12. [Testing Strategy](#12-testing-strategy)
13. [GUI Interface Contract Appendix](#13-gui-interface-contract-appendix)
14. [CAN Protocol Details (MIT Mini Cheetah)](#14-can-protocol-details-mit-mini-cheetah)
15. [Modbus RTU Protocol Details (BLD-3055)](#15-modbus-rtu-protocol-details-bld-3055)
16. [Full Directory Tree](#16-full-directory-tree)

---

## 1. System Overview

```
┌─────────────────────────────────────────────────────────────┐
│                      BASE STATION                           │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐   │
│  │              Next.js GUI (:3000)                      │   │
│  │                                                       │   │
│  │  ┌─────────────┐  ┌──────────────┐  ┌────────────┐  │   │
│  │  │ Connection   │  │ Status       │  │ Camera     │  │   │
│  │  │ Manager      │  │ Dashboard    │  │ Feeds      │  │   │
│  │  └──────┬───────┘  └──────┬───────┘  └─────┬──────┘  │   │
│  │         │                 │                 │         │   │
│  │  ┌──────┴───────┐  ┌─────┴────────┐  ┌────┴───────┐ │   │
│  │  │ Node Control │  │ Emergency    │  │ Joystick   │ │   │
│  │  │ Panel        │  │ Stop         │  │ Display    │ │   │
│  │  └──────┬───────┘  └──────┬───────┘  └────┬───────┘ │   │
│  │         │                 │                │         │   │
│  │         └────────┬────────┴────────┬───────┘         │   │
│  │                  │                 │                  │   │
│  │          ┌───────┴────────┐  ┌─────┴──────────┐      │   │
│  │          │ roslib (WS)    │  │ HTTP (MJPEG)   │      │   │
│  │          │ :9090          │  │ :8080           │      │   │
│  │          └───────┬────────┘  └─────┬──────────┘      │   │
│  └──────────────────┼────────────────┼───────────────────┘  │
└─────────────────────┼────────────────┼──────────────────────┘
                      │   WiFi / LAN   │
┌─────────────────────┼────────────────┼──────────────────────┐
│                     │     ROVER      │                       │
│           ┌─────────┴──────┐  ┌──────┴──────────┐           │
│           │ rosbridge_     │  │ web_video_       │           │
│           │ server (:9090) │  │ server (:8080)   │           │
│           └─────────┬──────┘  └──────┬──────────┘           │
│                     │                │                       │
│    ┌────────────────┴────────────────┴──────────────┐       │
│    │              ROS 2 Topic Bus                    │       │
│    └──┬───────┬───────┬───────┬───────┬───────┬─────┘       │
│       │       │       │       │       │       │             │
│  ┌────┴──┐┌───┴──┐┌───┴──┐┌──┴───┐┌──┴───┐┌──┴────┐       │
│  │rover_ ││rover_││rover_││rover_││rover_ ││rover_  │       │
│  │camera ││can   ││serial││drive ││monitor││estop   │       │
│  └───────┘└──┬───┘└──┬───┘└──────┘└───────┘└────────┘       │
│              │       │                                       │
│         ┌────┴───┐ ┌─┴────────┐                              │
│         │ CAN Bus│ │RS485 Bus │                              │
│         └────┬───┘ └─┬────────┘                              │
│              │       │                                       │
│  ┌───────────┴───────┴──────────────────────────────┐       │
│  │                  HARDWARE                         │       │
│  │  ┌──────────┐  ┌──────────────┐  ┌────────────┐  │       │
│  │  │ GIM Arm  │  │ BLD-3055     │  │ USB        │  │       │
│  │  │ Motors   │  │ Drive Motors │  │ Cameras x3 │  │       │
│  │  │ (CAN)    │  │ (RS485)      │  │            │  │       │
│  │  └──────────┘  └──────────────┘  └────────────┘  │       │
│  └──────────────────────────────────────────────────┘       │
└─────────────────────────────────────────────────────────────┘
```

### Communication Summary

| Path | Protocol | Port | Purpose |
|---|---|---|---|
| GUI ↔ rosbridge | WebSocket (JSON) | 9090 | Topics, services, parameters |
| GUI ↔ web_video_server | HTTP (MJPEG) | 8080 | Camera streams |
| Rover ↔ Arm motors | CAN (MIT Mini Cheetah) | — | 6-DOF joint control |
| Rover ↔ Drive motors | RS485 (Modbus RTU) | — | 6-wheel speed/direction |
| Rover ↔ Cameras | USB (UVC) | — | 3x 5MP video feeds |

---

## 2. Package Map

14 packages to create in `rover_ws/src/`:

| # | Package | Language | Purpose |
|---|---|---|---|
| 1 | `rover_bringup` | Python (launch only) | Launch files, config YAML, system startup |
| 2 | `rover_interfaces` | CMake (interfaces) | Custom message and service definitions |
| 3 | `rover_estop` | Python | Emergency stop coordinator |
| 4 | `rover_can` | C++ | CAN bus driver for arm (MIT Mini Cheetah) |
| 5 | `rover_serial` | C++ | RS485/Modbus RTU driver for drivetrain |
| 6 | `rover_camera` | Python | USB camera publisher (3x UVC cameras) |
| 7 | `rover_monitor` | Python | System telemetry (power, network, node list) |
| 8 | `rover_node_manager` | Python | GUI node launch/stop service handler |
| 9 | `rover_drive` | Python | Skid steer mixing (cmd_vel → motor speeds) |
| 10 | `rover_arm` | Python | 6-DOF joint control with safety limits |
| 11 | `rover_joy` | Python | Joystick input → cmd_vel / arm commands |
| 12 | `rover_autonomy` | Python | Waypoint navigation for mapping task |
| 13 | `rover_status_lights` | Python | Competition LED indicators |
| 14 | `rover_description` | — (URDF/xacro) | Robot model, TF publishers |

---

## 3. Node Specifications

### 3.1 `rover_estop` — Emergency Stop Coordinator

**Node name:** `estop_node`
**Language:** Python
**Package:** `rover_estop`

The e-stop node is the highest-priority safety node. When triggered, it commands all actuators to zero and prevents further motion until explicitly reset.

| Direction | Name | Type | Description |
|---|---|---|---|
| **Sub** | `/emergency_stop` | `std_msgs/msg/Bool` | `data: true` triggers e-stop (published by GUI) |
| **Pub** | `/estop/active` | `std_msgs/msg/Bool` | Current e-stop state (latched) |
| **Pub** | `/cmd_vel` | `geometry_msgs/msg/Twist` | Publishes zero twist on e-stop |
| **Pub** | `/arm/joint_commands` | `sensor_msgs/msg/JointState` | Publishes zero velocities on e-stop |
| **Srv** | `/estop/reset` | `std_srvs/srv/Trigger` | Reset e-stop (re-enables motion) |

**Parameters:**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `watchdog_timeout_sec` | double | 2.0 | If no heartbeat from GUI, auto-trigger e-stop |
| `require_manual_reset` | bool | true | E-stop latches until `/estop/reset` is called |

**Behavior:**
1. Subscribes to `/emergency_stop`. On `data: true`, sets internal `estop_active = true`.
2. While active: publishes zero `Twist` to `/cmd_vel` at 50 Hz and zero `JointState` to `/arm/joint_commands`.
3. Ignores all `/cmd_vel` and joint commands from other nodes (other nodes check `/estop/active` before publishing).
4. `/estop/reset` service sets `estop_active = false` only if `require_manual_reset` is true. Otherwise, e-stop auto-resets when `/emergency_stop` receives `data: false`.
5. Watchdog: if no message on `/emergency_stop` for `watchdog_timeout_sec`, auto-triggers (connection loss safety).

---

### 3.2 `rover_can` — CAN Bridge

**Node name:** `can_bridge_node`
**Language:** C++
**Package:** `rover_can`
**Executable:** `can_bridge_node`

Provides low-level CAN bus communication with the arm's GIM motors using the MIT Mini Cheetah protocol.

| Direction | Name | Type | Description |
|---|---|---|---|
| **Sub** | `/arm/joint_commands` | `sensor_msgs/msg/JointState` | Target positions/velocities for 6 joints |
| **Pub** | `/arm/joint_states` | `sensor_msgs/msg/JointState` | Feedback from motor encoders |
| **Pub** | `/system/can_status` | `std_msgs/msg/Bool` | `true` = CAN bus healthy |
| **Srv** | `/can/enable_motors` | `std_srvs/srv/SetBool` | Enable/disable all motors |

**Parameters:**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `can_interface` | string | `can0` | SocketCAN interface name |
| `bus_bitrate` | int | 1000000 | CAN bus bitrate (1 Mbps) |
| `motor_config_file` | string | `""` | Path to motor config YAML |
| `feedback_rate_hz` | double | 100.0 | Joint state publish rate |
| `status_rate_hz` | double | 2.0 | CAN status publish rate |

**Motor Config YAML format** (`config/arm_motors.yaml`):

```yaml
motors:
  - id: 1
    name: base_yaw
    type: GIM4305-10
    can_id: 0x01
    position_min: -3.14    # rad
    position_max: 3.14
    velocity_max: 10.0     # rad/s
    torque_max: 1.2        # Nm
    kp_default: 20.0
    kd_default: 1.0
  - id: 2
    name: shoulder_pitch
    type: GIM8108-48
    can_id: 0x02
    position_min: -1.57
    position_max: 1.57
    velocity_max: 5.0
    torque_max: 12.0
    kp_default: 50.0
    kd_default: 5.0
  - id: 3
    name: elbow_pitch
    type: GIM8108-48
    can_id: 0x03
    position_min: -2.35
    position_max: 2.35
    velocity_max: 5.0
    torque_max: 12.0
    kp_default: 50.0
    kd_default: 5.0
  - id: 4
    name: wrist_roll
    type: GIM4310-36
    can_id: 0x04
    position_min: -3.14
    position_max: 3.14
    velocity_max: 8.0
    torque_max: 3.6
    kp_default: 15.0
    kd_default: 1.0
  - id: 5
    name: wrist_pitch
    type: GIM4310-36
    can_id: 0x05
    position_min: -1.57
    position_max: 1.57
    velocity_max: 8.0
    torque_max: 3.6
    kp_default: 15.0
    kd_default: 1.0
  - id: 6
    name: gripper
    type: GIM4305-10
    can_id: 0x06
    position_min: 0.0
    position_max: 1.57
    velocity_max: 10.0
    torque_max: 1.2
    kp_default: 20.0
    kd_default: 1.0
```

**Behavior:**
1. Opens SocketCAN interface on startup. Publishes `can_status: true` if successful.
2. Sends MIT Mini Cheetah enable command to each motor on `/can/enable_motors` `data: true`.
3. Reads joint commands, converts to CAN frames, sends to motors.
4. Reads CAN feedback frames, publishes as `JointState` messages.
5. Monitors CAN bus health — publishes `false` on `/system/can_status` if read errors exceed threshold.

---

### 3.3 `rover_serial` — RS485 Bridge

**Node name:** `rs485_bridge_node`
**Language:** C++
**Package:** `rover_serial`
**Executable:** `rs485_bridge_node`

Provides Modbus RTU communication with the 6x BLD-3055 BLDC drive motors via RS485.

| Direction | Name | Type | Description |
|---|---|---|---|
| **Sub** | `/drive/motor_speeds` | `std_msgs/msg/Float64MultiArray` | Target speeds for 6 motors (RPM) |
| **Pub** | `/drive/motor_feedback` | `std_msgs/msg/Float64MultiArray` | Actual motor speeds (RPM) |
| **Pub** | `/system/rs485_status` | `std_msgs/msg/Bool` | `true` = RS485 bus healthy |

**Parameters:**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `serial_port` | string | `/dev/ttyUSB0` | RS485 serial port |
| `baudrate` | int | 9600 | Modbus RTU baud rate |
| `motor_ids` | int[] | `[1,2,3,4,5,6]` | Modbus slave addresses |
| `command_rate_hz` | double | 20.0 | Command send rate |
| `status_rate_hz` | double | 2.0 | RS485 status publish rate |
| `max_rpm` | int | 3000 | Maximum allowed RPM |

**Behavior:**
1. Opens serial port, verifies communication with each motor by reading status register.
2. Publishes `rs485_status: true` if at least one motor responds.
3. Receives `Float64MultiArray` with 6 values (left-front, left-mid, left-rear, right-front, right-mid, right-rear).
4. Converts RPM values to Modbus frames, sends to each motor sequentially.
5. Reads feedback registers, publishes actual RPM values.
6. On communication failure with a motor, logs error but continues with remaining motors.

---

### 3.4 `rover_camera` — Camera Publisher

**Node name:** `camera_node`
**Language:** Python
**Package:** `rover_camera`
**Executable:** `camera_node`

Publishes 3 USB camera feeds as ROS image topics. A 4th virtual topic is available for future expansion (gripper camera).

| Direction | Name | Type | Description |
|---|---|---|---|
| **Pub** | `/camera1/image_raw` | `sensor_msgs/msg/Image` | Front camera |
| **Pub** | `/camera2/image_raw` | `sensor_msgs/msg/Image` | Rear camera |
| **Pub** | `/camera3/image_raw` | `sensor_msgs/msg/Image` | Arm camera |
| **Pub** | `/camera1/camera_info` | `sensor_msgs/msg/CameraInfo` | Front camera calibration |
| **Pub** | `/camera2/camera_info` | `sensor_msgs/msg/CameraInfo` | Rear camera calibration |
| **Pub** | `/camera3/camera_info` | `sensor_msgs/msg/CameraInfo` | Arm camera calibration |

**Parameters:**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `camera_devices` | string[] | `["/dev/video0", "/dev/video2", "/dev/video4"]` | V4L2 device paths |
| `frame_rate` | double | 15.0 | Target FPS per camera |
| `resolution_width` | int | 640 | Capture width |
| `resolution_height` | int | 480 | Capture height |
| `camera_names` | string[] | `["camera1", "camera2", "camera3"]` | Frame IDs / topic prefixes |

**Behavior:**
1. Opens each V4L2 device using OpenCV `VideoCapture`.
2. Publishes `Image` messages on `/{name}/image_raw` at the configured frame rate.
3. Publishes empty `CameraInfo` (populated from calibration files if available).
4. If a camera device fails to open, logs a warning and skips it (does not crash).
5. The 4th camera slot (`/camera4/image_raw`) in the GUI is reserved — the rover does not currently have a 4th camera, but the topic will be served by `web_video_server` if a 4th device is added later.

**Note:** The GUI does NOT subscribe to these image topics via rosbridge. Instead, `web_video_server` reads these topics and serves MJPEG streams over HTTP on port 8080. The GUI fetches from `http://{roverIP}:8080/stream?topic=/camera1/image_raw&type=mjpeg&quality=80`.

---

### 3.5 `rover_monitor` — System Monitor

**Node name:** `system_monitor_node`
**Language:** Python
**Package:** `rover_monitor`
**Executable:** `system_monitor_node`

Publishes system telemetry that the GUI Status Dashboard consumes.

| Direction | Name | Type | Description |
|---|---|---|---|
| **Pub** | `/system/power` | `sensor_msgs/msg/BatteryState` | Battery voltage, current, percentage |
| **Pub** | `/system/network` | `std_msgs/msg/String` | JSON: `{"ssid","signal_strength","latency"}` |
| **Pub** | `/system/nodes` | `std_msgs/msg/String` | JSON: array of `{"id","name","status"}` |
| **Pub** | `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Standard diagnostics (optional) |

**Parameters:**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `power_topic_rate_hz` | double | 1.0 | Battery state publish rate |
| `network_topic_rate_hz` | double | 2.0 | Network status publish rate |
| `node_list_rate_hz` | double | 1.0 | Active node list publish rate |
| `battery_adc_device` | string | `""` | ADC device path for battery voltage |
| `battery_cells` | int | 6 | Number of LiPo cells (for % calculation) |
| `cell_voltage_min` | double | 3.3 | Empty cell voltage |
| `cell_voltage_max` | double | 4.2 | Full cell voltage |
| `ping_host` | string | `rover.local` | Host to ping for latency measurement |

**Behavior:**
1. **Power:** Reads battery voltage from ADC (or INA226 sensor via I2C). Computes percentage from cell count and voltage range. Publishes `BatteryState` with `voltage`, `current`, `percentage` fields.
2. **Network:** Runs `iwconfig`/`nmcli` to get SSID, signal strength (dBm → percentage). Pings base station for latency. Publishes JSON string:
   ```json
   {"ssid": "DeakinRover5G", "signal_strength": 75, "latency": 12.5}
   ```
3. **Node list:** Calls `ros2 node list` (via `rclpy` node graph API). Cross-references with expected nodes. Publishes JSON array:
   ```json
   [
     {"id": "camera_node", "name": "Camera Node", "status": "active"},
     {"id": "can_bridge", "name": "CAN Bridge", "status": "inactive"}
   ]
   ```
   The `id` values must match the `id` field in the GUI's `DEFAULT_NODES` constant.

---

### 3.6 `rover_node_manager` — Node Launch/Stop Service

**Node name:** `node_manager_node`
**Language:** Python
**Package:** `rover_node_manager`
**Executable:** `node_manager_node`

Provides ROS services for the GUI to launch and stop individual nodes remotely.

| Direction | Name | Type | Description |
|---|---|---|---|
| **Srv** | `/node_manager/launch` | `std_srvs/srv/Trigger` | Launch a node (request contains JSON in trigger) |
| **Srv** | `/node_manager/stop` | `std_srvs/srv/Trigger` | Stop a node (request contains JSON in trigger) |
| **Srv** | `/node_manager/list` | `std_srvs/srv/Trigger` | List managed nodes |

**Important — Service Interface Note:**

The GUI uses `std_srvs/srv/Trigger` (which has an empty request and returns `success: bool, message: string`). However, the GUI sends a `data` field containing JSON in the request:

- **Launch request:** `{ "data": "{\"node_id\":\"camera_node\",\"package\":\"rover_camera\",\"executable\":\"camera_node\"}" }`
- **Stop request:** `{ "data": "{\"node_id\":\"camera_node\"}" }`

Since `std_srvs/srv/Trigger` has no request fields, this `data` field will be silently ignored by the ROS service handler. The node manager must therefore determine which node to manage from context. There are two options:

**Option A (Recommended):** Create a custom service type in `rover_interfaces` that accepts a `string data` request field. Update the GUI's `SRV_TYPES.NODE_CONTROL` to match. This is a one-line change in the GUI's `constants.js`.

**Option B (Compatible):** Use `std_srvs/srv/SetBool` or create individual services per node (e.g., `/node_manager/launch/camera_node`). This avoids any GUI changes.

For now, the recommended approach is **Option A** — define a custom service:

```
# rover_interfaces/srv/NodeControl.srv
string data    # JSON: {"node_id": "...", "package": "...", "executable": "..."}
---
bool success
string message
```

And update GUI `constants.js`:
```js
NODE_CONTROL: 'rover_interfaces/srv/NodeControl',
```

**Behavior:**
1. Maintains a map of `node_id → subprocess` for managed nodes.
2. On `/node_manager/launch`: parses JSON from `data` field, runs `ros2 run {package} {executable}` as subprocess. Returns `success: true` on launch.
3. On `/node_manager/stop`: parses JSON from `data` field, sends SIGINT to subprocess. Returns `success: true` on clean shutdown.
4. On `/node_manager/list`: returns JSON array of managed node states in `message` field.
5. Monitors subprocesses — if a node crashes, updates its status to `"crashed"`.

---

### 3.7 `rover_drive` — Skid Steer Mixer

**Node name:** `drive_node`
**Language:** Python
**Package:** `rover_drive`
**Executable:** `drive_node`

Converts `geometry_msgs/msg/Twist` commands into individual motor speeds for 6-wheel skid steer.

| Direction | Name | Type | Description |
|---|---|---|---|
| **Sub** | `/cmd_vel` | `geometry_msgs/msg/Twist` | Linear x + angular z |
| **Sub** | `/estop/active` | `std_msgs/msg/Bool` | If true, zero all motors |
| **Pub** | `/drive/motor_speeds` | `std_msgs/msg/Float64MultiArray` | 6 motor speeds in RPM |

**Parameters:**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `wheel_separation` | double | 0.5 | Distance between left/right wheels (m) |
| `wheel_radius` | double | 0.1 | Wheel radius (m) |
| `max_linear_speed` | double | 1.0 | Max linear speed (m/s) |
| `max_angular_speed` | double | 2.0 | Max angular speed (rad/s) |
| `max_rpm` | int | 3000 | Maximum motor RPM |
| `cmd_vel_timeout_sec` | double | 0.5 | Zero motors if no cmd_vel received |
| `gear_ratio` | double | 1.0 | Motor-to-wheel gear ratio |

**Behavior:**
1. Receives `Twist` on `/cmd_vel`.
2. Skid steer mixing:
   ```
   left_speed  = linear.x - (angular.z * wheel_separation / 2)
   right_speed = linear.x + (angular.z * wheel_separation / 2)
   ```
3. Converts m/s → RPM: `rpm = (speed / (2π × wheel_radius)) × 60 × gear_ratio`
4. Clamps to `max_rpm`. All 3 left wheels get `left_rpm`, all 3 right wheels get `right_rpm`.
5. Publishes `Float64MultiArray`: `[left_front, left_mid, left_rear, right_front, right_mid, right_rear]`
6. If `/estop/active` is true or no `cmd_vel` for `cmd_vel_timeout_sec`, publishes all zeros.

---

### 3.8 `rover_arm` — 6-DOF Arm Controller

**Node name:** `arm_controller_node`
**Language:** Python
**Package:** `rover_arm`
**Executable:** `arm_controller_node`

High-level arm control: accepts joint-level commands, applies safety limits, and outputs to the CAN bridge.

| Direction | Name | Type | Description |
|---|---|---|---|
| **Sub** | `/arm/target_joints` | `sensor_msgs/msg/JointState` | Desired joint positions/velocities |
| **Sub** | `/arm/joint_states` | `sensor_msgs/msg/JointState` | Feedback from CAN bridge |
| **Sub** | `/estop/active` | `std_msgs/msg/Bool` | If true, zero all joints |
| **Pub** | `/arm/joint_commands` | `sensor_msgs/msg/JointState` | Clamped commands → CAN bridge |
| **Pub** | `/arm/status` | `std_msgs/msg/String` | JSON arm status |

**Parameters:**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `joint_names` | string[] | See motor config | Names matching motor config |
| `velocity_scale` | double | 0.5 | Scale factor for velocity commands (0-1) |
| `position_interpolation_rate_hz` | double | 50.0 | Smooth position interpolation rate |

**Behavior:**
1. Receives target joint positions/velocities on `/arm/target_joints`.
2. Clamps positions and velocities to limits from motor config YAML.
3. Optionally interpolates positions smoothly at 50 Hz for smooth motion.
4. Publishes clamped commands to `/arm/joint_commands` (consumed by `rover_can`).
5. Monitors feedback — if position error exceeds threshold, publishes warning.
6. On e-stop: publishes zero velocity commands.

---

### 3.9 `rover_joy` — Joystick Teleop

**Node name:** `joy_teleop_node`
**Language:** Python
**Package:** `rover_joy`
**Executable:** `joy_teleop_node`

Reads joystick input and produces drive/arm commands. Supports mode switching between drive and arm control.

| Direction | Name | Type | Description |
|---|---|---|---|
| **Sub** | `/joy` | `sensor_msgs/msg/Joy` | Joystick axes and buttons |
| **Sub** | `/estop/active` | `std_msgs/msg/Bool` | If true, suppress output |
| **Pub** | `/cmd_vel` | `geometry_msgs/msg/Twist` | Drive commands |
| **Pub** | `/arm/target_joints` | `sensor_msgs/msg/JointState` | Arm joint velocity commands |

**Parameters:**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `linear_axis` | int | 1 | Joystick axis for forward/backward |
| `angular_axis` | int | 0 | Joystick axis for turning |
| `linear_scale` | double | 1.0 | Linear speed multiplier |
| `angular_scale` | double | 2.0 | Angular speed multiplier |
| `mode_button` | int | 0 | Button to toggle drive/arm mode |
| `arm_axes` | int[] | `[0,1,2,3,4,5]` | Axes mapping for 6 arm joints |
| `arm_speed_scale` | double | 0.5 | Arm velocity multiplier |
| `deadzone` | double | 0.1 | Joystick deadzone threshold |

**Behavior:**
1. Subscribes to `/joy`. In drive mode (default), maps axes to `Twist` and publishes `/cmd_vel`.
2. `mode_button` toggles between drive and arm mode (with debounce).
3. In arm mode, maps joystick axes to joint velocities and publishes `/arm/target_joints`.
4. Applies deadzone filtering to all axes.
5. If e-stop is active, suppresses all output.

**Note:** The physical joystick driver (`joy_linux` or similar) runs on the base station and publishes `/joy`. This node on the rover subscribes to it via the ROS topic bridge. Alternatively, the joystick node can run on the base station with `/cmd_vel` relayed to the rover.

---

### 3.10 `rover_autonomy` — Waypoint Navigation

**Node name:** `autonomy_node`
**Language:** Python
**Package:** `rover_autonomy`
**Executable:** `autonomy_node`

Handles autonomous waypoint navigation for the mapping/traversal competition task.

| Direction | Name | Type | Description |
|---|---|---|---|
| **Sub** | `/gps/fix` | `sensor_msgs/msg/NavSatFix` | GPS position |
| **Sub** | `/imu/data` | `sensor_msgs/msg/Imu` | IMU orientation |
| **Sub** | `/estop/active` | `std_msgs/msg/Bool` | If true, halt navigation |
| **Pub** | `/cmd_vel` | `geometry_msgs/msg/Twist` | Drive commands |
| **Pub** | `/autonomy/status` | `std_msgs/msg/String` | JSON nav status |
| **Srv** | `/autonomy/set_waypoints` | `std_srvs/srv/Trigger` | Load waypoint list |
| **Srv** | `/autonomy/start` | `std_srvs/srv/Trigger` | Begin navigation |
| **Srv** | `/autonomy/stop` | `std_srvs/srv/Trigger` | Halt navigation |

**Parameters:**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `waypoint_file` | string | `""` | Path to waypoint CSV file |
| `arrival_radius_m` | double | 2.0 | Distance to consider waypoint reached |
| `max_speed` | double | 0.5 | Max autonomous speed (m/s) |
| `heading_kp` | double | 1.0 | Heading PID proportional gain |

**Behavior:**
1. Loads waypoints (lat, lon) from CSV or service call.
2. Uses GPS + IMU to compute heading and distance to next waypoint.
3. Simple heading-tracking controller outputs `Twist` commands.
4. Publishes status: `{"state": "navigating", "waypoint": 3, "total": 10, "distance": 5.2}`.
5. Halts on e-stop or `/autonomy/stop`.

---

### 3.11 `rover_status_lights` — Competition LED Indicators

**Node name:** `status_lights_node`
**Language:** Python
**Package:** `rover_status_lights`
**Executable:** `status_lights_node`

Controls competition-required LED status indicators via GPIO.

| Direction | Name | Type | Description |
|---|---|---|---|
| **Sub** | `/estop/active` | `std_msgs/msg/Bool` | Red LED on e-stop |
| **Sub** | `/autonomy/status` | `std_msgs/msg/String` | Blue LED during autonomous |
| **Pub** | (none) | | |

**Parameters:**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `gpio_red` | int | 17 | GPIO pin for red (e-stop) LED |
| `gpio_green` | int | 27 | GPIO pin for green (ready) LED |
| `gpio_blue` | int | 22 | GPIO pin for blue (autonomous) LED |

**Behavior:**
1. Red LED on when e-stop is active.
2. Green LED on when system is ready (e-stop inactive, motors enabled).
3. Blue LED on during autonomous navigation.
4. All LEDs off on shutdown.

---

### 3.12 `rover_description` — Robot Model

**Package:** `rover_description`
**Contents:** URDF/xacro files, meshes, launch files for `robot_state_publisher`.

No executable nodes — only data files and launch configuration.

**Files:**
- `urdf/rover.urdf.xacro` — Main robot model
- `urdf/arm.urdf.xacro` — Arm macro
- `urdf/drivetrain.urdf.xacro` — Wheel macro
- `meshes/` — STL files (if available)
- `launch/description.launch.py` — Launches `robot_state_publisher`
- `config/joint_limits.yaml` — Joint limits for arm

**TF Frames Published:**
- `base_link` → `base_footprint`
- `base_link` → `left_front_wheel`, `left_mid_wheel`, `left_rear_wheel`
- `base_link` → `right_front_wheel`, `right_mid_wheel`, `right_rear_wheel`
- `base_link` → `arm_base_link`
- `arm_base_link` → `shoulder_link` → `upper_arm_link` → `forearm_link` → `wrist_link` → `gripper_link`
- `base_link` → `camera1_link`, `camera2_link`, `camera3_link`

---

### 3.13 `rover_bringup` — Launch & Config

**Package:** `rover_bringup`
**Contents:** Launch files, config YAML, system startup scripts.

No executable nodes — orchestration only.

**Key files:**
- `launch/rover.launch.py` — Master launch file
- `launch/infrastructure.launch.py` — rosbridge, web_video_server
- `launch/hardware.launch.py` — CAN, RS485, cameras
- `launch/control.launch.py` — drive, arm, joy, estop
- `launch/monitoring.launch.py` — monitor, node_manager
- `config/arm_motors.yaml` — Motor configuration
- `config/drive_params.yaml` — Drivetrain parameters
- `config/camera_config.yaml` — Camera device mapping

---

### 3.14 `rover_interfaces` — Custom Messages & Services

**Package:** `rover_interfaces`
**Build type:** `ament_cmake` (required for interface generation)

**Custom service definitions:**

```
# srv/NodeControl.srv
string data    # JSON payload
---
bool success
string message
```

If additional custom messages are needed later (e.g., motor diagnostics), they go here.

---

## 4. Complete Topic Map

### 4.1 Topics Published by Rover Nodes

| # | Topic | Type | Publisher | Rate | Subscriber(s) |
|---|---|---|---|---|---|
| 1 | `/system/can_status` | `std_msgs/Bool` | `can_bridge_node` | 2 Hz | GUI |
| 2 | `/system/rs485_status` | `std_msgs/Bool` | `rs485_bridge_node` | 2 Hz | GUI |
| 3 | `/system/power` | `sensor_msgs/BatteryState` | `system_monitor_node` | 1 Hz | GUI |
| 4 | `/system/network` | `std_msgs/String` | `system_monitor_node` | 2 Hz | GUI |
| 5 | `/system/nodes` | `std_msgs/String` | `system_monitor_node` | 1 Hz | GUI |
| 6 | `/camera1/image_raw` | `sensor_msgs/Image` | `camera_node` | 15 Hz | web_video_server |
| 7 | `/camera2/image_raw` | `sensor_msgs/Image` | `camera_node` | 15 Hz | web_video_server |
| 8 | `/camera3/image_raw` | `sensor_msgs/Image` | `camera_node` | 15 Hz | web_video_server |
| 9 | `/camera1/camera_info` | `sensor_msgs/CameraInfo` | `camera_node` | 15 Hz | (calibration) |
| 10 | `/camera2/camera_info` | `sensor_msgs/CameraInfo` | `camera_node` | 15 Hz | (calibration) |
| 11 | `/camera3/camera_info` | `sensor_msgs/CameraInfo` | `camera_node` | 15 Hz | (calibration) |
| 12 | `/arm/joint_states` | `sensor_msgs/JointState` | `can_bridge_node` | 100 Hz | `arm_controller_node`, TF |
| 13 | `/arm/joint_commands` | `sensor_msgs/JointState` | `arm_controller_node` | 50 Hz | `can_bridge_node` |
| 14 | `/arm/status` | `std_msgs/String` | `arm_controller_node` | 5 Hz | GUI (future) |
| 15 | `/drive/motor_speeds` | `std_msgs/Float64MultiArray` | `drive_node` | 20 Hz | `rs485_bridge_node` |
| 16 | `/drive/motor_feedback` | `std_msgs/Float64MultiArray` | `rs485_bridge_node` | 20 Hz | `drive_node` (optional) |
| 17 | `/cmd_vel` | `geometry_msgs/Twist` | `joy_teleop_node` / `autonomy_node` / `estop_node` | 50 Hz | `drive_node` |
| 18 | `/arm/target_joints` | `sensor_msgs/JointState` | `joy_teleop_node` | 50 Hz | `arm_controller_node` |
| 19 | `/estop/active` | `std_msgs/Bool` | `estop_node` | 50 Hz | All motion nodes |
| 20 | `/autonomy/status` | `std_msgs/String` | `autonomy_node` | 1 Hz | GUI (future), `status_lights_node` |
| 21 | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `system_monitor_node` | 1 Hz | (standard diag) |

### 4.2 Topics Subscribed From External Sources

| # | Topic | Type | Publisher | Subscriber(s) |
|---|---|---|---|---|
| 22 | `/emergency_stop` | `std_msgs/Bool` | GUI (base station) | `estop_node` |
| 23 | `/joy` | `sensor_msgs/Joy` | Joystick driver (base station) | `joy_teleop_node`, GUI |
| 24 | `/gps/fix` | `sensor_msgs/NavSatFix` | GPS driver (future) | `autonomy_node` |
| 25 | `/imu/data` | `sensor_msgs/Imu` | IMU driver (future) | `autonomy_node` |

---

## 5. Complete Service Map

| # | Service | Type | Server Node | Client(s) |
|---|---|---|---|---|
| 1 | `/node_manager/launch` | `rover_interfaces/srv/NodeControl`* | `node_manager_node` | GUI |
| 2 | `/node_manager/stop` | `rover_interfaces/srv/NodeControl`* | `node_manager_node` | GUI |
| 3 | `/node_manager/list` | `std_srvs/srv/Trigger` | `node_manager_node` | GUI |
| 4 | `/can/enable_motors` | `std_srvs/srv/SetBool` | `can_bridge_node` | `node_manager_node` |
| 5 | `/estop/reset` | `std_srvs/srv/Trigger` | `estop_node` | GUI (future) |
| 6 | `/autonomy/set_waypoints` | `std_srvs/srv/Trigger` | `autonomy_node` | GUI (future) |
| 7 | `/autonomy/start` | `std_srvs/srv/Trigger` | `autonomy_node` | GUI (future) |
| 8 | `/autonomy/stop` | `std_srvs/srv/Trigger` | `autonomy_node` | GUI (future) |

\* See [Section 3.6](#36-rover_node_manager--node-launchstop-service) for the service type discussion. Requires either a custom `NodeControl.srv` or a GUI update.

---

## 6. Data Flow Diagrams

### 6.1 Joystick → Drivetrain

```
 Base Station                           Rover
┌───────────┐                  ┌──────────────────┐
│ Gamepad   │──USB──▶ joy_node │                  │
│           │        publishes │  /joy             │
└───────────┘        /joy      │    │              │
                               │    ▼              │
                               │ joy_teleop_node   │
                               │    │              │
                               │    ▼ /cmd_vel     │
                               │ drive_node        │
                               │    │              │
                               │    ▼ /drive/      │
                               │      motor_speeds │
                               │ rs485_bridge_node │
                               │    │              │
                               │    ▼ Modbus RTU   │
                               │ BLD-3055 Motors   │
                               └──────────────────┘
```

### 6.2 Joystick → Arm

```
 Base Station                           Rover
┌───────────┐                  ┌──────────────────────┐
│ Gamepad   │──USB──▶ joy_node │                      │
│ (arm mode)│        publishes │  /joy                 │
└───────────┘        /joy      │    │                  │
                               │    ▼                  │
                               │ joy_teleop_node       │
                               │    │                  │
                               │    ▼ /arm/            │
                               │      target_joints    │
                               │ arm_controller_node   │
                               │    │                  │
                               │    ▼ /arm/            │
                               │      joint_commands   │
                               │ can_bridge_node       │
                               │    │                  │
                               │    ▼ CAN Bus          │
                               │ GIM Motors            │
                               └──────────────────────┘
```

### 6.3 Camera → GUI

```
 Rover                                  Base Station
┌─────────────────────────┐    ┌────────────────────────┐
│ USB Cameras (x3)        │    │                        │
│    │                    │    │                        │
│    ▼                    │    │                        │
│ camera_node             │    │                        │
│    │ /camera{1,2,3}/    │    │                        │
│    │ image_raw          │    │                        │
│    ▼                    │    │                        │
│ web_video_server (:8080)│    │ Next.js GUI            │
│    │                    │──HTTP──▶  <img> tags        │
│    │ MJPEG stream       │ MJPEG│  2x2 grid           │
└─────────────────────────┘    └────────────────────────┘
```

### 6.4 Telemetry → GUI

```
 Rover                                  Base Station
┌──────────────────────────┐   ┌────────────────────────┐
│ system_monitor_node      │   │                        │
│   │ /system/power        │   │                        │
│   │ /system/network      │──WS──▶ Status Dashboard   │
│   │ /system/nodes        │ :9090│  (rosbridge)        │
│   │                      │   │                        │
│ can_bridge_node          │   │                        │
│   │ /system/can_status   │──WS──▶ CAN indicator      │
│   │                      │   │                        │
│ rs485_bridge_node        │   │                        │
│   │ /system/rs485_status │──WS──▶ RS485 indicator    │
└──────────────────────────┘   └────────────────────────┘
```

### 6.5 Emergency Stop Chain

```
 Base Station                           Rover
┌──────────────────┐           ┌────────────────────────────┐
│ GUI: E-Stop      │           │                            │
│ button / SPACE   │           │                            │
│    │             │           │                            │
│    ▼             │           │                            │
│ Publish          │──WS──▶   │ estop_node                 │
│ /emergency_stop  │ :9090    │    │                        │
│ {data: true}     │           │    ├──▶ /estop/active=true │
└──────────────────┘           │    │                        │
                               │    ├──▶ /cmd_vel = zero    │
                               │    │                        │
                               │    └──▶ /arm/joint_commands │
                               │         = zero              │
                               │                            │
                               │ All motion nodes check     │
                               │ /estop/active before       │
                               │ publishing commands        │
                               └────────────────────────────┘
```

---

## 7. TF Tree

```
                    base_footprint
                         │
                    base_link
                    ┌────┼─────────────────────────────────────┐
                    │    │    │    │    │    │    │    │    │   │
                   LF   LM   LR   RF   RM   RR  cam1 cam2 cam3
                 wheel wheel wheel wheel wheel wheel link link link
                    │
               arm_base_link
                    │
              shoulder_link
                    │
             upper_arm_link
                    │
              forearm_link
                    │
               wrist_link
                    │
              gripper_link

Legend:
  LF = left_front_wheel    RF = right_front_wheel
  LM = left_mid_wheel      RM = right_mid_wheel
  LR = left_rear_wheel     RR = right_rear_wheel
```

**Publishers:**
- `robot_state_publisher` — static transforms (URDF) for all fixed joints
- `joint_state_publisher` — arm joint states → TF (reads `/arm/joint_states`)
- `static_transform_publisher` — any ad-hoc static frames

---

## 8. Launch File Structure

### 8.1 Master Launch: `rover.launch.py`

```python
# rover_bringup/launch/rover.launch.py
# Launches all sub-launch files in dependency order

def generate_launch_description():
    return LaunchDescription([
        # Layer 1: Infrastructure (must start first)
        IncludeLaunchDescription('infrastructure.launch.py'),

        # Layer 2: Hardware drivers
        IncludeLaunchDescription('hardware.launch.py'),

        # Layer 3: Control nodes
        IncludeLaunchDescription('control.launch.py'),

        # Layer 4: Monitoring & management
        IncludeLaunchDescription('monitoring.launch.py'),
    ])
```

### 8.2 Infrastructure Layer: `infrastructure.launch.py`

| Node | Package | Notes |
|---|---|---|
| `rosbridge_websocket` | `rosbridge_server` | Port 9090 |
| `web_video_server` | `web_video_server` | Port 8080 |
| `robot_state_publisher` | `robot_state_publisher` | Loads URDF |
| `joint_state_publisher` | `joint_state_publisher` | Reads `/arm/joint_states` |

### 8.3 Hardware Layer: `hardware.launch.py`

| Node | Package | Notes |
|---|---|---|
| `can_bridge_node` | `rover_can` | Loads `arm_motors.yaml` |
| `rs485_bridge_node` | `rover_serial` | Loads `drive_params.yaml` |
| `camera_node` | `rover_camera` | Loads `camera_config.yaml` |

### 8.4 Control Layer: `control.launch.py`

| Node | Package | Notes |
|---|---|---|
| `estop_node` | `rover_estop` | Must start before motion nodes |
| `drive_node` | `rover_drive` | Depends on RS485 bridge |
| `arm_controller_node` | `rover_arm` | Depends on CAN bridge |
| `joy_teleop_node` | `rover_joy` | Depends on drive & arm |

### 8.5 Monitoring Layer: `monitoring.launch.py`

| Node | Package | Notes |
|---|---|---|
| `system_monitor_node` | `rover_monitor` | Publishes telemetry |
| `node_manager_node` | `rover_node_manager` | GUI node control |
| `status_lights_node` | `rover_status_lights` | Competition LEDs |

### 8.6 Launch Arguments

The master launch file should expose these arguments:

| Argument | Default | Description |
|---|---|---|
| `use_sim` | `false` | Launch in simulation mode (mock hardware) |
| `rosbridge_port` | `9090` | WebSocket port |
| `video_port` | `8080` | Web video server port |
| `can_interface` | `can0` | CAN bus interface |
| `serial_port` | `/dev/ttyUSB0` | RS485 serial port |
| `enable_autonomy` | `false` | Launch autonomy node |
| `enable_arm` | `true` | Launch arm nodes |

---

## 9. Implementation Phases

### Phase 1: Foundation (Weeks 1-2)

**Goal:** Rosbridge + cameras visible in GUI.

| Task | Package | Deliverable |
|---|---|---|
| Create workspace scaffold | All | `colcon build` succeeds with empty packages |
| `rover_interfaces` | `rover_interfaces` | `NodeControl.srv` builds |
| `rover_bringup` infrastructure launch | `rover_bringup` | rosbridge + web_video_server start |
| `rover_camera` basic | `rover_camera` | 3 cameras publish `/camera{1,2,3}/image_raw` |
| `rover_estop` | `rover_estop` | E-stop from GUI works |

**Verification:** GUI connects, camera feeds display, e-stop button works.

### Phase 2: Telemetry & Node Management (Weeks 3-4)

**Goal:** Full Status Dashboard functional.

| Task | Package | Deliverable |
|---|---|---|
| `rover_monitor` | `rover_monitor` | Power, network, node list topics |
| `rover_node_manager` | `rover_node_manager` | Launch/stop services work from GUI |
| Update GUI `constants.js` | (base station) | `NODE_CONTROL` type → `rover_interfaces/srv/NodeControl` |

**Verification:** Status Dashboard shows all indicators, node start/stop works from GUI.

### Phase 3: Drivetrain (Weeks 5-7)

**Goal:** Drive the rover with a joystick.

| Task | Package | Deliverable |
|---|---|---|
| `rover_serial` | `rover_serial` | Modbus RTU communication with BLD-3055 |
| `rover_drive` | `rover_drive` | Skid steer mixing |
| `rover_joy` (drive mode) | `rover_joy` | Joystick → `/cmd_vel` → motors spin |

**Verification:** Joystick drives rover forward/backward/turn. RS485 status shows green in GUI.

### Phase 4: Arm (Weeks 8-10)

**Goal:** Teleop the arm with joystick.

| Task | Package | Deliverable |
|---|---|---|
| `rover_can` | `rover_can` | CAN communication, MIT Mini Cheetah protocol |
| `rover_arm` | `rover_arm` | Safety limits, joint control |
| `rover_joy` (arm mode) | `rover_joy` | Mode switch, arm teleop |
| `rover_description` | `rover_description` | URDF, TF tree |

**Verification:** Joystick controls arm joints. CAN status shows green in GUI. TF tree visible in RViz.

### Phase 5: Autonomy & Polish (Weeks 11-12)

**Goal:** Autonomous navigation, competition readiness.

| Task | Package | Deliverable |
|---|---|---|
| `rover_autonomy` | `rover_autonomy` | GPS waypoint navigation |
| `rover_status_lights` | `rover_status_lights` | Competition LEDs |
| Integration testing | All | Full system test |
| `rover_bringup` systemd service | `rover_bringup` | Auto-start on boot |

**Verification:** Rover navigates waypoints autonomously. All LEDs indicate correct state.

---

## 10. 4-Wheel Steering Upgrade Path

When upgrading from 6-wheel skid steer to 4-wheel independent steering:

### What Changes

| Component | Current (6-wheel skid) | Future (4-wheel steer) |
|---|---|---|
| Motors | 6x BLD-3055 (drive only) | 4x drive + 4x steering |
| `rover_serial` | 6 motor IDs | 8 motor IDs (4 drive + 4 steer) |
| `rover_drive` | Skid steer mixing | Ackermann / swerve kinematics |
| `/drive/motor_speeds` | 6-element array | 4 drive + 4 steer angles |
| URDF | Fixed wheel orientation | Revolute steering joints |

### What Stays the Same

- `/cmd_vel` interface (Twist) — unchanged
- All GUI topics and services — unchanged
- CAN bridge (arm) — unchanged
- Camera, monitoring, e-stop — unchanged
- Joystick mapping — unchanged (outputs `Twist`)

### Migration Steps

1. **New message type:** Create `rover_interfaces/msg/SteerCommand.msg` with drive speeds and steer angles.
2. **Update `rover_drive`:** Replace skid steer mixing with swerve/Ackermann kinematics. Accept `Twist`, output `SteerCommand`.
3. **Update `rover_serial`:** Accept `SteerCommand`, send to 8 Modbus motors.
4. **Update URDF:** Add steering joints to wheel model.
5. **Update `drive_params.yaml`:** Add steering motor IDs, angle limits, PID gains.

The key design principle: **`/cmd_vel` is the abstraction boundary.** Everything above it (joystick, autonomy, GUI) is unchanged. Everything below it (drive node, serial bridge, motors) adapts to the new hardware.

---

## 11. Dependencies

### 11.1 APT Packages

```bash
# ROS 2 packages
sudo apt install -y \
  ros-jazzy-rosbridge-server \
  ros-jazzy-web-video-server \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joy-linux \
  ros-jazzy-diagnostic-updater \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport

# System packages
sudo apt install -y \
  can-utils \
  python3-opencv \
  python3-smbus \
  libmodbus-dev \
  i2c-tools
```

### 11.2 Python Packages

```bash
pip3 install \
  pymodbus>=3.0 \
  python-can>=4.0 \
  psutil \
  netifaces
```

### 11.3 Dockerfile Additions (Rover Container)

Add to `.devcontainer/rover/Dockerfile`:

```dockerfile
# CAN utilities
RUN apt-get update && apt-get install -y \
    can-utils \
    python3-opencv \
    python3-smbus \
    libmodbus-dev \
    i2c-tools \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 packages
RUN apt-get update && apt-get install -y \
    ros-jazzy-rosbridge-server \
    ros-jazzy-web-video-server \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joy-linux \
    ros-jazzy-diagnostic-updater \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    && rm -rf /var/lib/apt/lists/*

# Python packages
RUN pip3 install --no-cache-dir \
    pymodbus>=3.0 \
    python-can>=4.0 \
    psutil \
    netifaces
```

### 11.4 CAN Bus Setup

SocketCAN must be configured on the Jetson/RPi before `rover_can` starts:

```bash
# Load CAN kernel module
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mcp251x  # or peak_usb, depending on CAN adapter

# Configure CAN interface
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# Verify
candump can0  # should show frames if motors are connected
```

Add a systemd service or udev rule to auto-configure on boot:

```ini
# /etc/systemd/system/can0.service
[Unit]
Description=Configure CAN0 interface
After=network.target

[Service]
Type=oneshot
ExecStart=/sbin/ip link set can0 type can bitrate 1000000
ExecStartPost=/sbin/ip link set can0 up
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

---

## 12. Testing Strategy

### 12.1 Mock Nodes for Desktop Development

Create a `rover_bringup/launch/sim.launch.py` that launches mock versions of hardware nodes:

| Real Node | Mock Replacement | Behavior |
|---|---|---|
| `can_bridge_node` | `mock_can_node` (Python) | Echoes joint commands as joint states, always publishes `can_status: true` |
| `rs485_bridge_node` | `mock_serial_node` (Python) | Echoes motor speeds as feedback, always publishes `rs485_status: true` |
| `camera_node` | `mock_camera_node` (Python) | Publishes test pattern images |

This allows full GUI testing without physical hardware.

### 12.2 Unit Tests

| Package | Test Framework | What to Test |
|---|---|---|
| `rover_drive` | `pytest` + `rclpy` | Skid steer math (linear/angular → RPM) |
| `rover_arm` | `pytest` + `rclpy` | Joint limit clamping, safety checks |
| `rover_estop` | `pytest` + `rclpy` | Latch/reset behavior, watchdog |
| `rover_monitor` | `pytest` | JSON format correctness |
| `rover_can` | `gtest` | CAN frame encoding/decoding |
| `rover_serial` | `gtest` | Modbus frame encoding/decoding |

### 12.3 Integration Tests

```bash
# Test 1: GUI connectivity
# Start: rosbridge + web_video_server + mock nodes
# Verify: GUI connects, shows all status indicators green

# Test 2: E-stop chain
# Start: full mock system
# Action: Publish /emergency_stop {data: true}
# Verify: /cmd_vel goes to zero, /estop/active is true

# Test 3: Joystick → drive
# Start: full mock system + joy_node
# Action: Move joystick
# Verify: /cmd_vel has correct values, /drive/motor_speeds changes

# Test 4: Camera feeds
# Start: mock cameras + web_video_server
# Verify: GUI shows test patterns in camera grid
```

### 12.4 CLI Verification Commands

```bash
# Check all topics are published
ros2 topic list | grep -E "(system|camera|drive|arm|estop)"

# Monitor GUI-critical topics
ros2 topic echo /system/can_status
ros2 topic echo /system/rs485_status
ros2 topic echo /system/power
ros2 topic echo /system/network
ros2 topic echo /system/nodes

# Test e-stop
ros2 topic pub /emergency_stop std_msgs/msg/Bool "{data: true}" --once

# Test node manager service
ros2 service call /node_manager/launch rover_interfaces/srv/NodeControl \
  "{data: '{\"node_id\":\"camera_node\",\"package\":\"rover_camera\",\"executable\":\"camera_node\"}'}"

# Check TF tree
ros2 run tf2_tools view_frames

# Check camera feeds
ros2 topic hz /camera1/image_raw

# Monitor motor commands
ros2 topic echo /drive/motor_speeds
ros2 topic echo /arm/joint_commands
```

---

## 13. GUI Interface Contract Appendix

This is the definitive list of every interface the rover must satisfy for the existing GUI to work correctly. Derived from the GUI source code.

### 13.1 Required Topic Subscriptions (GUI subscribes, rover publishes)

| Topic | Type | JSON Payload | GUI Component |
|---|---|---|---|
| `/system/can_status` | `std_msgs/msg/Bool` | `{data: true/false}` | StatusDashboard CAN indicator |
| `/system/rs485_status` | `std_msgs/msg/Bool` | `{data: true/false}` | StatusDashboard RS485 indicator |
| `/system/power` | `sensor_msgs/msg/BatteryState` | `{voltage, current, percentage}` | StatusDashboard power card |
| `/joy` | `sensor_msgs/msg/Joy` | `{axes: [...], buttons: [...]}` | StatusDashboard joystick indicator |
| `/system/network` | `std_msgs/msg/String` | `{data: '{"ssid":"...","signal_strength":75,"latency":12.5}'}` | StatusDashboard network card |
| `/system/nodes` | `std_msgs/msg/String` | `{data: '[{"id":"camera_node","name":"Camera Node","status":"active"},...]'}` | StatusDashboard node list |

### 13.2 Required Topic Publications (GUI publishes, rover subscribes)

| Topic | Type | Payload | GUI Trigger |
|---|---|---|---|
| `/emergency_stop` | `std_msgs/msg/Bool` | `{data: true}` | E-Stop button or SPACE key |

### 13.3 Required Services (GUI calls, rover serves)

| Service | Type | Request | Response |
|---|---|---|---|
| `/node_manager/launch` | `std_srvs/srv/Trigger`* | `{data: '{"node_id":"...","package":"...","executable":"..."}'}` | `{success: true/false, message: "..."}` |
| `/node_manager/stop` | `std_srvs/srv/Trigger`* | `{data: '{"node_id":"..."}'}` | `{success: true/false, message: "..."}` |
| `/node_manager/list` | `std_srvs/srv/Trigger` | `{}` | `{success: true/false, message: "..."}` |

\* See naming discrepancy note below.

### 13.4 Required Camera Topics (via web_video_server HTTP, not rosbridge)

| Topic | Stream URL Pattern |
|---|---|
| `/camera1/image_raw` | `http://{roverIP}:{videoPort}/stream?topic=/camera1/image_raw&type=mjpeg&quality={q}&width={w}&height={h}` |
| `/camera2/image_raw` | Same pattern with `/camera2/image_raw` |
| `/camera3/image_raw` | Same pattern with `/camera3/image_raw` |
| `/camera4/image_raw` | Same pattern with `/camera4/image_raw` (reserved, no physical camera yet) |

**Stream defaults:** `type=mjpeg`, `quality=80`, `width=640`, `height=480`

### 13.5 Required Infrastructure

| Service | Port | Package |
|---|---|---|
| rosbridge WebSocket server | 9090 (configurable) | `rosbridge_server` |
| web_video_server HTTP | 8080 (configurable) | `web_video_server` |

### 13.6 DEFAULT_NODES in GUI (constants.js)

These are the nodes the GUI knows about for the Node Control panel:

| id | name | package | executable |
|---|---|---|---|
| `camera_node` | Camera Node | `rover_camera` | `camera_node` |
| `motor_ctrl` | Motor Controller | `rover_motors` | `motor_controller` |
| `can_bridge` | CAN Bridge | `rover_can` | `can_bridge_node` |
| `rs485_bridge` | RS485 Bridge | `rover_serial` | `rs485_bridge_node` |
| `sensor_hub` | Sensor Hub | `rover_sensors` | `sensor_hub_node` |
| `system_monitor` | System Monitor | `rover_monitor` | `system_monitor_node` |

### 13.7 Naming Discrepancy — Action Required

The GUI's `DEFAULT_NODES` references two packages/executables that don't exist in this architecture:

| GUI Reference | Issue | Recommended Fix |
|---|---|---|
| Package `rover_motors`, executable `motor_controller` | Architecture uses `rover_drive` / `drive_node` | Update GUI `constants.js`: `package: 'rover_drive'`, `executable: 'drive_node'` |
| Package `rover_sensors`, executable `sensor_hub_node` | Architecture uses `rover_monitor` / `system_monitor_node` | Update GUI `constants.js`: `package: 'rover_monitor'`, `executable: 'system_monitor_node'` (or remove duplicate since `system_monitor` entry already exists) |

Additionally, the service type discrepancy:

| GUI Constant | Current Value | Recommended Value |
|---|---|---|
| `SRV_TYPES.NODE_CONTROL` | `std_srvs/srv/Trigger` | `rover_interfaces/srv/NodeControl` |

These are small GUI-side changes (3-4 lines in `constants.js`) that should be made in Phase 2 when the node manager is implemented.

---

## 14. CAN Protocol Details (MIT Mini Cheetah)

### 14.1 Overview

The Steadywin GIM motors use the MIT Mini Cheetah protocol over CAN 2.0A (standard 11-bit ID). Each motor has a unique CAN ID (1-6).

### 14.2 Command Frame Format (Rover → Motor)

| Byte(s) | Field | Bits | Range | Description |
|---|---|---|---|---|
| 0-1 | Position | 16 | [-π, π] or [-12.5, 12.5] | Target position (rad) |
| 2-3 (upper 12) | Velocity | 12 | [-velocity_max, velocity_max] | Target velocity (rad/s) |
| 3-4 (lower 12) | Kp | 12 | [0, 500] | Position gain |
| 5-6 (upper 12) | Kd | 12 | [0, 100] | Velocity/damping gain |
| 6-7 (lower 12) | Torque | 12 | [-torque_max, torque_max] | Feedforward torque (Nm) |

**CAN ID:** Motor ID (1-6) in the standard 11-bit arbitration field.

**Total:** 8 bytes per frame.

### 14.3 Feedback Frame Format (Motor → Rover)

| Byte(s) | Field | Bits | Description |
|---|---|---|---|
| 0-1 | Position | 16 | Current position (rad) |
| 2-3 (upper 12) | Velocity | 12 | Current velocity (rad/s) |
| 3-4 (lower 12) | Current/Torque | 12 | Current torque (Nm) |

**CAN ID:** Motor ID + offset (typically motor_id for replies).

### 14.4 Value Encoding

All values are encoded as unsigned integers mapped linearly to their float range:

```
uint_value = (float_value - min) / (max - min) * ((1 << bits) - 1)
float_value = uint_value / ((1 << bits) - 1) * (max - min) + min
```

Example for position (16-bit, range [-π, π]):
```
uint16 = (position_rad + π) / (2π) * 65535
position_rad = uint16 / 65535 * 2π - π
```

### 14.5 Special Commands

| Command | CAN Data (8 bytes) | Description |
|---|---|---|
| Enable motor | `0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFC` | Enter motor control mode |
| Disable motor | `0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFD` | Exit motor control mode (safe) |
| Zero position | `0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFE` | Set current position as zero |

### 14.6 Motor Configuration Summary

| Joint | Motor | CAN ID | τ_max (Nm) | ω_max (rad/s) | Gear Ratio |
|---|---|---|---|---|---|
| Base Yaw | GIM4305-10 | 0x01 | 1.2 | 10.0 | 10:1 |
| Shoulder Pitch | GIM8108-48 | 0x02 | 12.0 | 5.0 | 48:1 |
| Elbow Pitch | GIM8108-48 | 0x03 | 12.0 | 5.0 | 48:1 |
| Wrist Roll | GIM4310-36 | 0x04 | 3.6 | 8.0 | 36:1 |
| Wrist Pitch | GIM4310-36 | 0x05 | 3.6 | 8.0 | 36:1 |
| Gripper | GIM4305-10 | 0x06 | 1.2 | 10.0 | 10:1 |

---

## 15. Modbus RTU Protocol Details (BLD-3055)

### 15.1 Overview

The BLD-3055 BLDC motor drivers communicate via RS485 using Modbus RTU protocol. Each motor has a unique slave address (1-6).

### 15.2 Communication Parameters

| Parameter | Value |
|---|---|
| Baud rate | 9600 (default, configurable to 19200/38400) |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Protocol | Modbus RTU |

### 15.3 Key Registers

| Register | Address | R/W | Description |
|---|---|---|---|
| Control word | 0x2000 | W | Motor control (start/stop/direction) |
| Target speed | 0x2001 | W | Target speed (RPM, 0-3000) |
| Actual speed | 0x2002 | R | Current speed (RPM) |
| Motor status | 0x2003 | R | Status flags |
| Bus voltage | 0x2004 | R | DC bus voltage (0.1V units) |
| Motor current | 0x2005 | R | Phase current (0.01A units) |
| Error code | 0x2006 | R | Error flags |
| Acceleration | 0x2007 | W | Acceleration time (ms, 0-65535) |
| Deceleration | 0x2008 | W | Deceleration time (ms, 0-65535) |

### 15.4 Control Word (0x2000)

| Bit | Value | Description |
|---|---|---|
| 0 | 0/1 | Stop / Run |
| 1 | 0/1 | Forward / Reverse |
| 2 | 0/1 | Normal / Brake |

Examples:
- `0x0001` = Run forward
- `0x0003` = Run reverse
- `0x0000` = Stop (coast)
- `0x0004` = Brake

### 15.5 Modbus RTU Frame Format

**Write single register (Function 0x06):**

```
[Slave Addr] [0x06] [Reg Hi] [Reg Lo] [Value Hi] [Value Lo] [CRC Lo] [CRC Hi]
```

Example — Set motor 1 speed to 1500 RPM:
```
01 06 20 01 05 DC XX XX
│  │  │     │     └── CRC-16
│  │  │     └── 1500 = 0x05DC
│  │  └── Register 0x2001
│  └── Function: Write Single Register
└── Slave address 1
```

**Read holding register (Function 0x03):**

```
[Slave Addr] [0x03] [Start Hi] [Start Lo] [Count Hi] [Count Lo] [CRC Lo] [CRC Hi]
```

Example — Read motor 1 actual speed:
```
01 03 20 02 00 01 XX XX
```

Response:
```
01 03 02 05 DC XX XX
│  │  │  │     └── CRC-16
│  │  │  └── 1500 = 0x05DC (actual RPM)
│  │  └── 2 bytes follow
│  └── Function: Read Holding Registers
└── Slave address 1
```

### 15.6 Motor Wiring (6-Wheel Skid Steer)

| Modbus ID | Position | Direction Convention |
|---|---|---|
| 1 | Left Front | Forward = CW |
| 2 | Left Mid | Forward = CW |
| 3 | Left Rear | Forward = CW |
| 4 | Right Front | Forward = CCW (reversed) |
| 5 | Right Mid | Forward = CCW (reversed) |
| 6 | Right Rear | Forward = CCW (reversed) |

**Note:** Right-side motors spin in the opposite direction for forward motion. The `rover_serial` node handles this direction inversion.

---

## 16. Full Directory Tree

After all packages are implemented:

```
rover_ws/
├── ros_architecture.md          ← This document
├── src/
│   ├── rover_bringup/
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── launch/
│   │   │   ├── rover.launch.py
│   │   │   ├── infrastructure.launch.py
│   │   │   ├── hardware.launch.py
│   │   │   ├── control.launch.py
│   │   │   ├── monitoring.launch.py
│   │   │   └── sim.launch.py
│   │   ├── config/
│   │   │   ├── arm_motors.yaml
│   │   │   ├── drive_params.yaml
│   │   │   └── camera_config.yaml
│   │   └── scripts/
│   │       └── setup_can.sh
│   │
│   ├── rover_interfaces/
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   └── srv/
│   │       └── NodeControl.srv
│   │
│   ├── rover_estop/
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── rover_estop/
│   │   │   ├── __init__.py
│   │   │   └── estop_node.py
│   │   └── test/
│   │       └── test_estop.py
│   │
│   ├── rover_can/
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── include/rover_can/
│   │   │   ├── can_bridge_node.hpp
│   │   │   └── mini_cheetah_protocol.hpp
│   │   ├── src/
│   │   │   ├── can_bridge_node.cpp
│   │   │   └── mini_cheetah_protocol.cpp
│   │   └── test/
│   │       └── test_protocol.cpp
│   │
│   ├── rover_serial/
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── include/rover_serial/
│   │   │   ├── rs485_bridge_node.hpp
│   │   │   └── modbus_protocol.hpp
│   │   ├── src/
│   │   │   ├── rs485_bridge_node.cpp
│   │   │   └── modbus_protocol.cpp
│   │   └── test/
│   │       └── test_modbus.cpp
│   │
│   ├── rover_camera/
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── rover_camera/
│   │   │   ├── __init__.py
│   │   │   └── camera_node.py
│   │   └── test/
│   │       └── test_camera.py
│   │
│   ├── rover_monitor/
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── rover_monitor/
│   │   │   ├── __init__.py
│   │   │   └── system_monitor_node.py
│   │   └── test/
│   │       └── test_monitor.py
│   │
│   ├── rover_node_manager/
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── rover_node_manager/
│   │   │   ├── __init__.py
│   │   │   └── node_manager_node.py
│   │   └── test/
│   │       └── test_node_manager.py
│   │
│   ├── rover_drive/
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── rover_drive/
│   │   │   ├── __init__.py
│   │   │   └── drive_node.py
│   │   └── test/
│   │       └── test_skid_steer.py
│   │
│   ├── rover_arm/
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── rover_arm/
│   │   │   ├── __init__.py
│   │   │   └── arm_controller_node.py
│   │   └── test/
│   │       └── test_arm_limits.py
│   │
│   ├── rover_joy/
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── rover_joy/
│   │   │   ├── __init__.py
│   │   │   └── joy_teleop_node.py
│   │   └── test/
│   │       └── test_joy_mapping.py
│   │
│   ├── rover_autonomy/
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── rover_autonomy/
│   │   │   ├── __init__.py
│   │   │   └── autonomy_node.py
│   │   └── test/
│   │       └── test_waypoint_nav.py
│   │
│   ├── rover_status_lights/
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── rover_status_lights/
│   │   │   ├── __init__.py
│   │   │   └── status_lights_node.py
│   │   └── test/
│   │       └── test_lights.py
│   │
│   └── rover_description/
│       ├── package.xml
│       ├── CMakeLists.txt
│       ├── urdf/
│       │   ├── rover.urdf.xacro
│       │   ├── arm.urdf.xacro
│       │   └── drivetrain.urdf.xacro
│       ├── meshes/
│       │   └── .gitkeep
│       ├── config/
│       │   └── joint_limits.yaml
│       └── launch/
│           └── description.launch.py
│
└── .gitkeep
```

---

## Appendix A: Quick Reference Card

### Minimum Viable System (Phase 1)

To get the GUI fully connected with basic functionality, you need:

1. `rosbridge_server` running on port 9090
2. `web_video_server` running on port 8080
3. `camera_node` publishing `/camera{1,2,3}/image_raw`
4. `system_monitor_node` publishing `/system/power`, `/system/network`, `/system/nodes`
5. `can_bridge_node` publishing `/system/can_status`
6. `rs485_bridge_node` publishing `/system/rs485_status`
7. `estop_node` subscribing to `/emergency_stop`
8. `node_manager_node` serving `/node_manager/launch` and `/node_manager/stop`

### Build & Run Cheat Sheet

```bash
# Build all packages
cd ~/rover_ws
colcon build --symlink-install

# Source workspace
source install/setup.bash

# Launch everything
ros2 launch rover_bringup rover.launch.py

# Launch with simulation (mock hardware)
ros2 launch rover_bringup rover.launch.py use_sim:=true

# Launch only infrastructure (for GUI testing)
ros2 launch rover_bringup infrastructure.launch.py
```
