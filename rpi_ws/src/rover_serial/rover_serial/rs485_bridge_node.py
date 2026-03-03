#!/usr/bin/env python3
"""
RS485 Bridge Node — Modbus RTU communication with BLD-3055 drive motors.

6-wheel skid steer: 6x BLD-3055 BLDC motors connected via RS485 bus.

Motor wiring (Modbus slave addresses):
  ID 1: Left Front   (forward = CW)
  ID 2: Left Mid     (forward = CW)
  ID 3: Left Rear    (forward = CW)
  ID 4: Right Front  (forward = CCW — reversed for forward motion)
  ID 5: Right Mid    (forward = CCW)
  ID 6: Right Rear   (forward = CCW)

BLD-3055 Modbus registers:
  0x2000: Control word (bit0=run, bit1=direction, bit2=brake)
  0x2001: Target speed (RPM, 0-3000)
  0x2002: Actual speed (RPM, read-only)

MOCK MODE (Orange Pi / no RS485 hardware):
  Simulates motor response with first-order lag.
  Always reports RS485 as healthy.

HARDWARE UPGRADE (Jetson/RPi with RS485 adapter):
  1. Set parameter use_mock=false
  2. Set serial_port to your RS485 device (e.g., '/dev/ttyUSB0')
  3. Install pymodbus: pip3 install pymodbus
  4. Verify: python3 -c "from pymodbus.client import ModbusSerialClient; print('OK')"

TOPICS:
  Sub: /drive/motor_speeds (std_msgs/Float64MultiArray) — 6 target RPM values
  Pub: /drive/motor_feedback (std_msgs/Float64MultiArray) — 6 actual RPM values
  Pub: /system/rs485_status (std_msgs/Bool) — bus health for GUI dashboard

PARAMETERS:
  use_mock (bool, default=true)
  serial_port (string, default='/dev/ttyUSB0')
  baudrate (int, default=9600)
  motor_ids (int[], default=[1,2,3,4,5,6])
  command_rate_hz (double, default=20.0)
  status_rate_hz (double, default=2.0)
  max_rpm (int, default=3000)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool

# Optional hardware import
try:
    from pymodbus.client import ModbusSerialClient
    HAS_PYMODBUS = True
except ImportError:
    HAS_PYMODBUS = False


class RS485BridgeNode(Node):
    """RS485/Modbus RTU bridge for BLD-3055 drive motors."""

    # BLD-3055 register addresses
    REG_CONTROL = 0x2000
    REG_TARGET_SPEED = 0x2001
    REG_ACTUAL_SPEED = 0x2002

    # Control word bits
    CTRL_RUN = 0x0001
    CTRL_REVERSE = 0x0002
    CTRL_BRAKE = 0x0004

    # Right-side motors need reversed direction
    REVERSED_MOTORS = {4, 5, 6}

    def __init__(self):
        super().__init__('rs485_bridge_node')

        # -- Parameters --
        self.declare_parameter('use_mock', True)
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('motor_ids', [1, 2, 3, 4, 5, 6])
        self.declare_parameter('command_rate_hz', 20.0)
        self.declare_parameter('status_rate_hz', 2.0)
        self.declare_parameter('max_rpm', 3000)

        self.use_mock = self.get_parameter('use_mock').value
        self.motor_ids = self.get_parameter('motor_ids').value
        self.max_rpm = self.get_parameter('max_rpm').value

        # -- State --
        self.modbus_client = None
        self.bus_healthy = False
        self.target_speeds = [0.0] * 6   # RPM (signed: + = forward, - = reverse)
        self.actual_speeds = [0.0] * 6   # RPM (simulated feedback)

        # -- Subscribers --
        self.create_subscription(
            Float64MultiArray, '/drive/motor_speeds', self._on_speeds, 10)

        # -- Publishers --
        self.pub_feedback = self.create_publisher(
            Float64MultiArray, '/drive/motor_feedback', 10)
        self.pub_status = self.create_publisher(Bool, '/system/rs485_status', 10)

        # -- Timers --
        cmd_period = 1.0 / self.get_parameter('command_rate_hz').value
        st_period = 1.0 / self.get_parameter('status_rate_hz').value
        self.create_timer(cmd_period, self._command_loop)
        self.create_timer(st_period, self._publish_status)

        # -- Init --
        if self.use_mock:
            self.bus_healthy = True
            self.get_logger().info('RS485 bridge started in MOCK mode')
        else:
            self._init_modbus()

    def _init_modbus(self):
        """Initialize Modbus RTU serial connection.

        HARDWARE UPGRADE: Runs on real hardware with pymodbus installed.
        """
        if not HAS_PYMODBUS:
            self.get_logger().error(
                'pymodbus not installed! Install with: pip3 install pymodbus')
            return

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baudrate').value

        try:
            self.modbus_client = ModbusSerialClient(
                port=port, baudrate=baud,
                parity='N', stopbits=1, bytesize=8, timeout=0.5)

            if self.modbus_client.connect():
                self.bus_healthy = True
                self.get_logger().info(f'Modbus connected: {port} @ {baud} baud')
            else:
                self.get_logger().error(f'Failed to connect to {port}')
        except Exception as e:
            self.get_logger().error(f'Modbus init failed: {e}')

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #

    def _on_speeds(self, msg: Float64MultiArray):
        """Receive target motor speeds (RPM, signed)."""
        data = list(msg.data)
        # Pad/trim to 6 motors
        data += [0.0] * (6 - len(data))
        self.target_speeds = data[:6]

    # ------------------------------------------------------------------ #
    # Control Loop
    # ------------------------------------------------------------------ #

    def _command_loop(self):
        """Send speed commands and read feedback."""
        if self.use_mock:
            # Simulate first-order motor dynamics (tau ~ 100ms)
            alpha = 0.2  # Response factor per tick
            for i in range(6):
                self.actual_speeds[i] += alpha * (self.target_speeds[i] - self.actual_speeds[i])
        else:
            self._send_modbus_commands()
            self._read_modbus_feedback()

        # Publish feedback
        feedback = Float64MultiArray()
        feedback.data = list(self.actual_speeds)
        self.pub_feedback.publish(feedback)

    def _send_modbus_commands(self):
        """Send speed commands to each motor via Modbus RTU.

        HARDWARE UPGRADE: This is the real Modbus communication code.
        For each motor:
          1. Write control word (run + direction)
          2. Write target speed (absolute RPM)
        """
        if not self.modbus_client:
            return

        for i, motor_id in enumerate(self.motor_ids):
            rpm = self.target_speeds[i]
            abs_rpm = min(abs(rpm), self.max_rpm)

            # Determine direction (right-side motors are physically reversed)
            forward = rpm >= 0
            if motor_id in self.REVERSED_MOTORS:
                forward = not forward

            # Control word
            ctrl = self.CTRL_RUN if abs_rpm > 0 else 0
            if not forward:
                ctrl |= self.CTRL_REVERSE

            try:
                self.modbus_client.write_register(
                    self.REG_CONTROL, ctrl, slave=motor_id)
                self.modbus_client.write_register(
                    self.REG_TARGET_SPEED, int(abs_rpm), slave=motor_id)
            except Exception as e:
                self.get_logger().debug(f'Modbus write error motor {motor_id}: {e}')

    def _read_modbus_feedback(self):
        """Read actual speed from each motor.

        HARDWARE UPGRADE: Reads register 0x2002 from each motor.
        """
        if not self.modbus_client:
            return

        for i, motor_id in enumerate(self.motor_ids):
            try:
                result = self.modbus_client.read_holding_registers(
                    self.REG_ACTUAL_SPEED, 1, slave=motor_id)
                if result and not result.isError():
                    self.actual_speeds[i] = float(result.registers[0])
                    # Re-apply sign based on target direction
                    if self.target_speeds[i] < 0:
                        self.actual_speeds[i] = -self.actual_speeds[i]
            except Exception as e:
                self.get_logger().debug(f'Modbus read error motor {motor_id}: {e}')

    def _publish_status(self):
        """Publish RS485 bus health for GUI dashboard."""
        msg = Bool(data=self.bus_healthy)
        self.pub_status.publish(msg)

    def destroy_node(self):
        """Stop all motors and close connection."""
        if self.modbus_client:
            # Stop all motors
            for motor_id in self.motor_ids:
                try:
                    self.modbus_client.write_register(
                        self.REG_CONTROL, 0, slave=motor_id)
                except Exception:
                    pass
            self.modbus_client.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RS485BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
