#!/usr/bin/env python3
"""
Skid Steer Mixer Node — converts Twist commands to 6 motor speeds.

Takes geometry_msgs/Twist on /cmd_vel and outputs 6 motor RPM values
as Float64MultiArray on /drive/motor_speeds.

Mixing equations (6-wheel skid steer):
  left_speed  = linear.x - (angular.z * wheel_separation / 2)
  right_speed = linear.x + (angular.z * wheel_separation / 2)

Motor speed array: [left_front, left_mid, left_rear, right_front, right_mid, right_rear]

Safety:
  - Checks /estop/active before publishing
  - Zeros all motors if no /cmd_vel received within timeout

TOPICS:
  Sub: /cmd_vel (geometry_msgs/Twist) — linear.x + angular.z
  Sub: /estop/active (std_msgs/Bool)  — suppress output when true
  Pub: /drive/motor_speeds (std_msgs/Float64MultiArray) — 6 RPM values

PARAMETERS:
  wheel_separation (double, default=0.5) — distance between left/right wheels (m)
  wheel_radius (double, default=0.1) — wheel radius (m)
  max_linear_speed (double, default=1.0) — max forward speed (m/s)
  max_angular_speed (double, default=2.0) — max turn rate (rad/s)
  max_rpm (int, default=3000) — motor RPM limit
  cmd_vel_timeout_sec (double, default=0.5) — zero if no cmd_vel received
  gear_ratio (double, default=1.0) — motor-to-wheel gear ratio

MOCK MODE: No hardware dependencies. Works on Orange Pi.
HARDWARE UPGRADE: No changes needed — output goes to rover_serial.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Bool


class DriveNode(Node):
    """Skid steer mixer: cmd_vel → motor speeds."""

    def __init__(self):
        super().__init__('drive_node')

        # -- Parameters --
        self.declare_parameter('wheel_separation', 0.5)
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('max_rpm', 3000)
        self.declare_parameter('cmd_vel_timeout_sec', 0.5)
        self.declare_parameter('gear_ratio', 1.0)

        self.wheel_sep = self.get_parameter('wheel_separation').value
        self.wheel_rad = self.get_parameter('wheel_radius').value
        self.max_lin = self.get_parameter('max_linear_speed').value
        self.max_ang = self.get_parameter('max_angular_speed').value
        self.max_rpm = self.get_parameter('max_rpm').value
        self.timeout = self.get_parameter('cmd_vel_timeout_sec').value
        self.gear_ratio = self.get_parameter('gear_ratio').value

        # -- State --
        self.estop_active = False
        self.last_cmd_time = self.get_clock().now()
        self.last_twist = Twist()

        # -- Subscribers --
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(Bool, '/estop/active', self._on_estop, 10)

        # -- Publishers --
        self.pub_speeds = self.create_publisher(
            Float64MultiArray, '/drive/motor_speeds', 10)

        # -- Timer: publish at 20 Hz --
        self.create_timer(0.05, self._publish_speeds)

        self.get_logger().info(
            f'Drive node ready (sep={self.wheel_sep}m, radius={self.wheel_rad}m, '
            f'max_rpm={self.max_rpm})')

    def _on_cmd_vel(self, msg: Twist):
        self.last_twist = msg
        self.last_cmd_time = self.get_clock().now()

    def _on_estop(self, msg: Bool):
        self.estop_active = msg.data

    def _publish_speeds(self):
        """Convert latest Twist to 6 motor RPM values."""
        # Check timeout
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        timed_out = elapsed > self.timeout

        if self.estop_active or timed_out:
            speeds = [0.0] * 6
        else:
            speeds = self._mix(self.last_twist)

        msg = Float64MultiArray()
        msg.data = speeds
        self.pub_speeds.publish(msg)

    def _mix(self, twist: Twist) -> list:
        """Skid steer mixing: Twist → 6 motor RPMs."""
        # Clamp inputs
        linear_x = max(-self.max_lin, min(self.max_lin, twist.linear.x))
        angular_z = max(-self.max_ang, min(self.max_ang, twist.angular.z))

        # Differential drive mixing
        left_vel = linear_x - (angular_z * self.wheel_sep / 2.0)   # m/s
        right_vel = linear_x + (angular_z * self.wheel_sep / 2.0)  # m/s

        # Convert m/s to RPM: rpm = (vel / (2π * radius)) * 60 * gear_ratio
        left_rpm = (left_vel / (2.0 * math.pi * self.wheel_rad)) * 60.0 * self.gear_ratio
        right_rpm = (right_vel / (2.0 * math.pi * self.wheel_rad)) * 60.0 * self.gear_ratio

        # Clamp to max RPM
        left_rpm = max(-self.max_rpm, min(self.max_rpm, left_rpm))
        right_rpm = max(-self.max_rpm, min(self.max_rpm, right_rpm))

        # All left wheels same speed, all right wheels same speed
        return [left_rpm, left_rpm, left_rpm, right_rpm, right_rpm, right_rpm]


def main(args=None):
    rclpy.init(args=args)
    node = DriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
