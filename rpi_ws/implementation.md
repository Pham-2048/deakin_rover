# Deakin Rover — Implementation Guide

Step-by-step instructions for building, testing, and deploying the rover ROS 2 system.

---

## Step 1: Build the Docker Container

On your **Raspberry Pi** (or any machine with Docker):

```bash
cd /path/to/deakin_rover
# Open in VS Code with the opiz devcontainer, OR build manually:
docker build -t rover-dev -f .devcontainer/opiz/Dockerfile .
```

The Dockerfile installs all ROS 2 dependencies:
- `ros-jazzy-rosbridge-server` (GUI WebSocket connection)
- `ros-jazzy-foxglove-bridge` (backup GUI)
- `ros-jazzy-web-video-server` (camera MJPEG streams)
- `ros-jazzy-robot-state-publisher`, `ros-jazzy-joint-state-publisher` (TF)
- `ros-jazzy-cv-bridge`, `ros-jazzy-xacro` (camera/URDF)
- `python3-opencv`, `psutil` (monitoring/camera)

---

## Step 2: Build the ROS 2 Workspace

Inside the container:

```bash
cd ~/rover_ws

# Build all packages
colcon build --symlink-install

# Source the workspace (also added to ~/.bashrc automatically)
source install/setup.bash
```

**Expected output:** All 14 packages build successfully with 0 errors.

If you get build errors:
- `rover_interfaces` must build first (it generates the NodeControl.srv type)
- Run `colcon build --packages-select rover_interfaces` first, then `source install/setup.bash`, then `colcon build`

---

## Step 3: Launch the Full System (Mock Mode)

```bash
# Launch everything in mock mode (default):
ros2 launch rover_bringup rover.launch.py
```

This starts 4 layers:
1. **Infrastructure:** rosbridge (:9090), Foxglove (:8765), web_video_server (:8080), TF
2. **Hardware:** Camera (test patterns), CAN bridge (mock), RS485 bridge (mock)
3. **Control:** E-stop, drive mixer, arm controller, joystick teleop
4. **Monitoring:** System monitor, node manager

---

## Step 4: Verify Topics

In a new terminal (source the workspace first):

```bash
source ~/rover_ws/install/setup.bash

# List all topics — should see ~20+ topics
ros2 topic list

# Verify GUI-critical topics are publishing:
ros2 topic echo /system/can_status --once       # Should show: data: true
ros2 topic echo /system/rs485_status --once      # Should show: data: true
ros2 topic echo /system/power --once             # Should show voltage/current/percentage
ros2 topic echo /system/network --once           # Should show JSON with ssid/signal/latency
ros2 topic echo /system/nodes --once             # Should show JSON array of node statuses
ros2 topic hz /camera1/image_raw                 # Should show ~15 Hz
```

---

## Step 5: Connect the GUI

On your **base station** (laptop):

1. Start the Next.js GUI:
   ```bash
   cd base_station_ws/gui
   npm run dev
   ```

2. Open `http://localhost:3000` in your browser

3. In the Connection Manager, enter:
   - **Rover IP:** Your Orange Pi's IP address (e.g., `192.168.1.100`)
   - **Port:** `9090`
   - **Video Port:** `8080`

4. You should see:
   - Status Dashboard: CAN ✅, RS485 ✅, battery data, network stats
   - Camera Feeds: Color-coded test patterns (blue/green/red)
   - Node Control: List of nodes with start/stop buttons
   - Emergency Stop: Click or press SPACE to trigger

---

## Step 6: Connect Foxglove Studio (Backup GUI)

1. Download [Foxglove Studio](https://foxglove.dev/download)
2. Open Foxglove → "Open connection"
3. Select "Foxglove WebSocket" → Enter: `ws://<rover_ip>:8765`
4. You'll see all topics, can add panels for:
   - Image panel → select `/camera1/image_raw`
   - Plot panel → select `/system/power`
   - 3D panel → shows URDF model with TF tree
   - Teleop panel → publishes `/joy` messages (can drive the rover!)

---

## Step 7: Test E-Stop

```bash
# From CLI:
ros2 topic pub /emergency_stop std_msgs/msg/Bool "{data: true}" --once

# Verify:
ros2 topic echo /estop/active --once    # Should show: data: true

# Reset:
ros2 service call /estop/reset std_srvs/srv/Trigger
```

Or click the E-STOP button in the GUI / press SPACE.

---

## Step 8: Test Node Manager

```bash
# Launch a node via service call (simulates GUI):
ros2 service call /node_manager/launch rover_interfaces/srv/NodeControl \
  "{data: '{\"node_id\":\"camera_node\",\"package\":\"rover_camera\",\"executable\":\"camera_node\"}'}"

# Stop it:
ros2 service call /node_manager/stop rover_interfaces/srv/NodeControl \
  "{data: '{\"node_id\":\"camera_node\"}'}"

# List managed nodes:
ros2 service call /node_manager/list std_srvs/srv/Trigger
```

---

## Deploying to Real Rover Hardware

When you move from Orange Pi to the actual rover (Jetson/RPi):

### Step A: Update Dockerfile for Real Hardware

Uncomment the hardware dependencies in `.devcontainer/opiz/Dockerfile`:

```dockerfile
# Remove the comment markers (#) from these blocks:
RUN sudo apt-get update && sudo apt-get install -y \
    can-utils \
    libmodbus-dev \
    i2c-tools \
    python3-smbus \
    && sudo rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir --break-system-packages \
    pymodbus>=3.0 \
    python-can>=4.0
```

### Step B: Configure CAN Bus

```bash
# Load CAN kernel module
sudo modprobe can
sudo modprobe can_raw

# Configure CAN interface (1 Mbps for MIT Mini Cheetah protocol)
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# Verify — should see frames if arm motors are powered
candump can0
```

### Step C: Find Camera Devices

```bash
# List USB cameras
v4l2-ctl --list-devices

# Example output:
#   LattePanda Camera (usb-0000:00:14.0-1):
#       /dev/video0
#       /dev/video1
```

Update `config/camera_config.yaml`:
```yaml
camera_node:
  ros__parameters:
    use_mock: false    # ← Change from true to false
    camera_devices:
      - "/dev/video0"  # ← Update with actual device paths
      - "/dev/video2"
      - "/dev/video4"
```

### Step D: Find RS485 Serial Port

```bash
# List serial devices
ls /dev/ttyUSB* /dev/ttyACM*

# Or check dmesg for USB-serial adapter
dmesg | grep ttyUSB
```

### Step E: Launch with Real Hardware

```bash
ros2 launch rover_bringup rover.launch.py use_mock:=false
```

---

## Files You'll Need to Edit

Here's a quick reference of all files containing `HARDWARE UPGRADE` comments.
Search for this string in any file to find what needs changing:

```bash
grep -r "HARDWARE UPGRADE" rover_ws/src/ --include="*.py" --include="*.yaml" -l
```

### Most Important Files to Edit:

| File | What to Change |
|---|---|
| `config/camera_config.yaml` | `use_mock: false`, camera device paths |
| `config/drive_params.yaml` | wheel_separation, wheel_radius, gear_ratio |
| `config/arm_motors.yaml` | Verify CAN IDs match your wiring |
| `launch/hardware.launch.py` | `use_mock` default value |
| `launch/monitoring.launch.py` | `use_mock` default value, `ping_host` |
| `launch/control.launch.py` | `watchdog_timeout_sec` (reduce for production) |

### Node Files with Hardware Sections:

| Node | What to Implement |
|---|---|
| `rover_can/can_bridge_node.py` | Real CAN read/write in `_send_can_commands()` |
| `rover_serial/rs485_bridge_node.py` | Real Modbus in `_send_modbus_commands()` |
| `rover_camera/camera_node.py` | Works automatically when `use_mock=false` |
| `rover_monitor/system_monitor_node.py` | Battery sensor in `_read_real_battery()` |
| `rover_status_lights/status_lights_node.py` | GPIO pins, `use_mock=false` |

---

## Launching Individual Layers

You don't have to launch everything at once:

```bash
# Just infrastructure (test GUI connection):
ros2 launch rover_bringup infrastructure.launch.py

# Just hardware:
ros2 launch rover_bringup hardware.launch.py use_mock:=true

# Just control:
ros2 launch rover_bringup control.launch.py

# Just monitoring:
ros2 launch rover_bringup monitoring.launch.py
```

---

## Troubleshooting

### "Package not found" errors
```bash
# Make sure workspace is sourced
source ~/rover_ws/install/setup.bash

# Rebuild if needed
cd ~/rover_ws && colcon build --symlink-install
```

### GUI can't connect
- Check rover IP: `hostname -I`
- Check rosbridge is running: `ros2 node list | grep rosbridge`
- Test WebSocket: `curl http://<rover_ip>:9090` (should show "Can 'Upgrade' only to 'WebSocket'")

### Camera feeds not showing in GUI
- Check web_video_server: `curl http://<rover_ip>:8080`
- Check camera topics: `ros2 topic hz /camera1/image_raw`
- Verify stream URL: `http://<rover_ip>:8080/stream?topic=/camera1/image_raw&type=mjpeg`

### web_video_server not available
If `ros-jazzy-web-video-server` fails to install, build from source:
```bash
cd ~/rover_ws/src
git clone https://github.com/RobotWebTools/web_video_server.git -b ros2
cd ~/rover_ws
colcon build --packages-select web_video_server
```

### E-stop watchdog keeps triggering
The watchdog auto-triggers if no `/emergency_stop` message is received. Either:
- Increase `watchdog_timeout_sec` in `control.launch.py`
- Or have the GUI connected (it should periodically publish to the topic)

### Foxglove can't connect
- Check foxglove_bridge node: `ros2 node list | grep foxglove`
- Verify port 8765 is open: `ss -tlnp | grep 8765`
- Try connecting to: `ws://<rover_ip>:8765`

---

## Package Summary

| Package | Node | Purpose |
|---|---|---|
| `rover_interfaces` | — | Custom `NodeControl.srv` service definition |
| `rover_bringup` | — | Launch files + config YAMLs |
| `rover_estop` | `estop_node` | Emergency stop coordinator |
| `rover_can` | `can_bridge_node` | CAN bus → arm motors |
| `rover_serial` | `rs485_bridge_node` | RS485/Modbus → drive motors |
| `rover_camera` | `camera_node` | USB camera publisher |
| `rover_monitor` | `system_monitor_node` | Power/network/node telemetry |
| `rover_node_manager` | `node_manager_node` | GUI node launch/stop services |
| `rover_drive` | `drive_node` | Skid steer mixer (cmd_vel → RPM) |
| `rover_arm` | `arm_controller_node` | Arm safety limits |
| `rover_joy` | `joy_teleop_node` | Joystick → drive/arm commands |
| `rover_autonomy` | `autonomy_node` | Waypoint navigation (stub) |
| `rover_status_lights` | `status_lights_node` | Competition LEDs (stub) |
| `rover_description` | — | URDF robot model + TF |

---

## Port Reference

| Port | Service | Protocol | Used By |
|---|---|---|---|
| 9090 | rosbridge_server | WebSocket | Next.js GUI (roslib) |
| 8765 | foxglove_bridge | WebSocket | Foxglove Studio |
| 8080 | web_video_server | HTTP/MJPEG | GUI camera feeds |
| 3000 | Next.js dev server | HTTP | Base station browser |
