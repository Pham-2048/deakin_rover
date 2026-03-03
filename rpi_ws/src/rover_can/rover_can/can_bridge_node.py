#!/usr/bin/env python3
"""
CAN Bridge Node — communicates with 6-DOF arm motors via CAN bus.

Uses MIT Mini Cheetah protocol with Steadywin GIM motors:
  - GIM4305-10 (joints 1, 6): base_yaw, gripper
  - GIM8108-48 (joints 2, 3): shoulder_pitch, elbow_pitch
  - GIM4310-36 (joints 4, 5): wrist_roll, wrist_pitch

MOCK MODE (Orange Pi / no CAN hardware):
  Echoes commanded positions as feedback with simulated dynamics.
  Always reports CAN bus as healthy.

HARDWARE UPGRADE (Jetson/RPi with CAN adapter):
  1. Set parameter use_mock=false
  2. Set can_interface to your SocketCAN interface (e.g., 'can0')
  3. Install python-can: pip3 install python-can
  4. Configure CAN interface:
       sudo ip link set can0 type can bitrate 1000000
       sudo ip link set can0 up
  5. Verify: candump can0

CAN FRAME FORMAT (MIT Mini Cheetah):
  Command (8 bytes): [pos_hi, pos_lo, vel_hi:4|vel_lo:8, kp_hi:4|kp_lo:8, kd_hi:4|kd_lo:8, torque_hi:4|torque_lo:8]
  Feedback (5 bytes): [pos_hi, pos_lo, vel_hi:4|vel_lo:8, current_hi:4|current_lo:8]
  Enable:  0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFC
  Disable: 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFD
  Zero:    0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFE

TOPICS:
  Sub: /arm/joint_commands (sensor_msgs/JointState) — target positions/velocities
  Pub: /arm/joint_states (sensor_msgs/JointState)   — encoder feedback
  Pub: /system/can_status (std_msgs/Bool)            — bus health for GUI dashboard

SERVICES:
  /can/enable_motors (std_srvs/SetBool) — enable/disable motor control mode

PARAMETERS:
  use_mock (bool, default=true)
  can_interface (string, default='can0')
  motor_config_file (string, default='') — path to arm_motors.yaml
  feedback_rate_hz (double, default=100.0)
  status_rate_hz (double, default=2.0)
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

# Optional hardware import
try:
    import can
    HAS_PYTHON_CAN = True
except ImportError:
    HAS_PYTHON_CAN = False


class CANBridgeNode(Node):
    """CAN bus bridge for arm motor communication."""

    JOINT_NAMES = [
        'base_yaw', 'shoulder_pitch', 'elbow_pitch',
        'wrist_roll', 'wrist_pitch', 'gripper'
    ]

    # MIT Mini Cheetah special commands
    CMD_ENABLE  = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
    CMD_DISABLE = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
    CMD_ZERO    = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE])

    def __init__(self):
        super().__init__('can_bridge_node')

        # -- Parameters --
        self.declare_parameter('use_mock', True)
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('motor_config_file', '')
        self.declare_parameter('feedback_rate_hz', 100.0)
        self.declare_parameter('status_rate_hz', 2.0)

        self.use_mock = self.get_parameter('use_mock').value
        self.can_interface_name = self.get_parameter('can_interface').value

        # -- State --
        self.motors_enabled = False
        self.can_bus = None
        self.can_healthy = False
        self.error_count = 0

        # Mock state: current positions track toward targets
        self.mock_positions = [0.0] * 6
        self.mock_velocities = [0.0] * 6
        self.target_positions = [0.0] * 6
        self.target_velocities = [0.0] * 6

        # -- Subscribers --
        self.create_subscription(
            JointState, '/arm/joint_commands', self._on_commands, 10)

        # -- Publishers --
        self.pub_states = self.create_publisher(JointState, '/arm/joint_states', 10)
        self.pub_status = self.create_publisher(Bool, '/system/can_status', 10)

        # -- Services --
        self.create_service(SetBool, '/can/enable_motors', self._on_enable)

        # -- Timers --
        fb_period = 1.0 / self.get_parameter('feedback_rate_hz').value
        st_period = 1.0 / self.get_parameter('status_rate_hz').value
        self.create_timer(fb_period, self._publish_feedback)
        self.create_timer(st_period, self._publish_status)

        # -- Init hardware --
        if self.use_mock:
            self.can_healthy = True
            self.get_logger().info('CAN bridge started in MOCK mode')
        else:
            self._init_can()

    def _init_can(self):
        """Initialize SocketCAN interface.

        HARDWARE UPGRADE: This runs on real hardware with python-can installed.
        """
        if not HAS_PYTHON_CAN:
            self.get_logger().error(
                'python-can not installed! Install with: pip3 install python-can')
            return

        try:
            self.can_bus = can.interface.Bus(
                channel=self.can_interface_name,
                bustype='socketcan',
                bitrate=1000000
            )
            self.can_healthy = True
            self.get_logger().info(f'CAN bus opened: {self.can_interface_name}')
        except Exception as e:
            self.can_healthy = False
            self.get_logger().error(f'Failed to open CAN bus: {e}')

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #

    def _on_commands(self, msg: JointState):
        """Receive joint commands from arm controller."""
        if not self.motors_enabled:
            return

        positions = list(msg.position) if msg.position else [0.0] * 6
        velocities = list(msg.velocity) if msg.velocity else [0.0] * 6

        # Pad to 6 joints
        positions += [0.0] * (6 - len(positions))
        velocities += [0.0] * (6 - len(velocities))

        self.target_positions = positions[:6]
        self.target_velocities = velocities[:6]

        if not self.use_mock and self.can_bus:
            self._send_can_commands(positions, velocities)

    def _on_enable(self, request, response):
        """Enable or disable motor control mode."""
        self.motors_enabled = request.data
        action = 'enable' if request.data else 'disable'

        if self.use_mock:
            response.success = True
            response.message = f'Motors {action}d (mock)'
            self.get_logger().info(f'Motors {action}d (mock)')
        elif self.can_bus:
            try:
                cmd = self.CMD_ENABLE if request.data else self.CMD_DISABLE
                for motor_id in range(1, 7):
                    frame = can.Message(
                        arbitration_id=motor_id, data=cmd, is_extended_id=False)
                    self.can_bus.send(frame)
                response.success = True
                response.message = f'Motors {action}d'
                self.get_logger().info(f'Motors {action}d via CAN')
            except Exception as e:
                response.success = False
                response.message = f'Failed to {action} motors: {e}'
                self.get_logger().error(response.message)
        else:
            response.success = False
            response.message = 'CAN bus not initialized'

        return response

    # ------------------------------------------------------------------ #
    # Publishers
    # ------------------------------------------------------------------ #

    def _publish_feedback(self):
        """Publish joint states (motor encoder feedback)."""
        if self.use_mock:
            # Simulate motor dynamics: positions track toward targets
            dt = 1.0 / self.get_parameter('feedback_rate_hz').value
            for i in range(6):
                error = self.target_positions[i] - self.mock_positions[i]
                # Simple P-controller simulation
                self.mock_velocities[i] = max(-2.0, min(2.0, error * 5.0))
                self.mock_positions[i] += self.mock_velocities[i] * dt

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.JOINT_NAMES

        if self.use_mock:
            msg.position = list(self.mock_positions)
            msg.velocity = list(self.mock_velocities)
            msg.effort = [0.0] * 6
        else:
            # HARDWARE UPGRADE: Read CAN feedback frames here
            # feedback = self._read_can_feedback()
            msg.position = [0.0] * 6
            msg.velocity = [0.0] * 6
            msg.effort = [0.0] * 6

        self.pub_states.publish(msg)

    def _publish_status(self):
        """Publish CAN bus health status for GUI dashboard."""
        msg = Bool()
        msg.data = self.can_healthy
        self.pub_status.publish(msg)

    # ------------------------------------------------------------------ #
    # CAN Protocol (MIT Mini Cheetah)
    # ------------------------------------------------------------------ #

    def _send_can_commands(self, positions, velocities):
        """Encode and send MIT Mini Cheetah command frames.

        HARDWARE UPGRADE: This is the real CAN communication code.
        Each motor gets an 8-byte command frame with:
          - 16-bit position
          - 12-bit velocity
          - 12-bit Kp
          - 12-bit Kd
          - 12-bit feedforward torque
        """
        if not self.can_bus:
            return

        # TODO: Implement full MIT Mini Cheetah encoding
        # For now, send position-only commands
        # See ros_architecture.md Section 14 for frame format details
        for motor_id in range(1, 7):
            try:
                pos = positions[motor_id - 1]
                vel = velocities[motor_id - 1]

                # Simplified encoding — replace with full protocol
                data = self._encode_command(pos, vel, kp=20.0, kd=1.0, torque=0.0)

                frame = can.Message(
                    arbitration_id=motor_id,
                    data=data,
                    is_extended_id=False
                )
                self.can_bus.send(frame)
            except Exception as e:
                self.error_count += 1
                if self.error_count > 100:
                    self.can_healthy = False
                self.get_logger().debug(f'CAN send error motor {motor_id}: {e}')

    @staticmethod
    def _encode_command(position, velocity, kp, kd, torque,
                        p_min=-12.5, p_max=12.5,
                        v_min=-45.0, v_max=45.0,
                        kp_min=0.0, kp_max=500.0,
                        kd_min=0.0, kd_max=5.0,
                        t_min=-18.0, t_max=18.0):
        """Encode MIT Mini Cheetah command frame (8 bytes).

        All values are linearly mapped to unsigned integers:
          uint = (value - min) / (max - min) * max_uint
        """
        def float_to_uint(val, val_min, val_max, bits):
            val = max(val_min, min(val_max, val))
            span = val_max - val_min
            return int((val - val_min) / span * ((1 << bits) - 1))

        p_int = float_to_uint(position, p_min, p_max, 16)
        v_int = float_to_uint(velocity, v_min, v_max, 12)
        kp_int = float_to_uint(kp, kp_min, kp_max, 12)
        kd_int = float_to_uint(kd, kd_min, kd_max, 12)
        t_int = float_to_uint(torque, t_min, t_max, 12)

        data = bytes([
            (p_int >> 8) & 0xFF,
            p_int & 0xFF,
            (v_int >> 4) & 0xFF,
            ((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F),
            kp_int & 0xFF,
            (kd_int >> 4) & 0xFF,
            ((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F),
            t_int & 0xFF,
        ])
        return data

    def destroy_node(self):
        """Clean up CAN bus."""
        if self.can_bus:
            # Disable all motors before shutting down
            for motor_id in range(1, 7):
                try:
                    frame = can.Message(
                        arbitration_id=motor_id,
                        data=self.CMD_DISABLE,
                        is_extended_id=False)
                    self.can_bus.send(frame)
                except Exception:
                    pass
            self.can_bus.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CANBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
