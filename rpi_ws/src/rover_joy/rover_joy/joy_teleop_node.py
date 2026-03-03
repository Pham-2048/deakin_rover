#!/usr/bin/env python3
"""
Joystick Teleop Node — maps joystick input to drive/arm commands.

Supports two modes toggled by a button:
  DRIVE MODE (default): Left stick → linear/angular → /cmd_vel
  ARM MODE: Sticks → joint velocities → /arm/target_joints

The physical joystick driver (joy_linux) runs on the base station and
publishes /joy. This node on the rover subscribes via the ROS topic bridge.

TOPICS:
  Sub: /joy (sensor_msgs/Joy) — joystick axes and buttons
  Sub: /estop/active (std_msgs/Bool) — suppress all output when true
  Pub: /cmd_vel (geometry_msgs/Twist) — drive commands (drive mode)
  Pub: /arm/target_joints (sensor_msgs/JointState) — arm commands (arm mode)

PARAMETERS:
  linear_axis (int, default=1) — joystick axis for forward/backward
  angular_axis (int, default=0) — joystick axis for turning
  linear_scale (double, default=1.0) — max linear speed (m/s)
  angular_scale (double, default=2.0) — max angular speed (rad/s)
  mode_button (int, default=0) — button index to toggle drive/arm mode
  deadzone (double, default=0.1) — joystick deadzone threshold
  arm_speed_scale (double, default=0.5) — arm velocity multiplier

FOXGLOVE COMPATIBILITY:
  Foxglove can publish /joy messages via its Teleop panel, or you can use
  the ROS joy_linux node with a physical gamepad.

MOCK MODE: No hardware dependencies. Works on Orange Pi.
HARDWARE UPGRADE: No changes needed.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class JoyTeleopNode(Node):
    """Joystick teleop with drive/arm mode switching."""

    ARM_JOINT_NAMES = [
        'base_yaw', 'shoulder_pitch', 'elbow_pitch',
        'wrist_roll', 'wrist_pitch', 'gripper'
    ]

    def __init__(self):
        super().__init__('joy_teleop_node')

        # -- Parameters --
        self.declare_parameter('linear_axis', 1)
        self.declare_parameter('angular_axis', 0)
        self.declare_parameter('linear_scale', 1.0)
        self.declare_parameter('angular_scale', 2.0)
        self.declare_parameter('mode_button', 0)
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('arm_speed_scale', 0.5)

        self.lin_axis = self.get_parameter('linear_axis').value
        self.ang_axis = self.get_parameter('angular_axis').value
        self.lin_scale = self.get_parameter('linear_scale').value
        self.ang_scale = self.get_parameter('angular_scale').value
        self.mode_btn = self.get_parameter('mode_button').value
        self.deadzone = self.get_parameter('deadzone').value
        self.arm_scale = self.get_parameter('arm_speed_scale').value

        # -- State --
        self.drive_mode = True  # True = drive, False = arm
        self.estop_active = False
        self.prev_mode_btn = 0  # For debounce

        # -- Subscribers --
        self.create_subscription(Joy, '/joy', self._on_joy, 10)
        self.create_subscription(Bool, '/estop/active', self._on_estop, 10)

        # -- Publishers --
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_arm = self.create_publisher(
            JointState, '/arm/target_joints', 10)

        mode_str = 'DRIVE' if self.drive_mode else 'ARM'
        self.get_logger().info(
            f'Joy teleop ready (mode={mode_str}, '
            f'linear_axis={self.lin_axis}, angular_axis={self.ang_axis}, '
            f'mode_button={self.mode_btn})')

    def _apply_deadzone(self, value):
        """Apply deadzone filtering."""
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def _on_estop(self, msg: Bool):
        self.estop_active = msg.data

    def _on_joy(self, msg: Joy):
        """Handle joystick input."""
        if self.estop_active:
            return

        # Mode toggle (button press with debounce)
        if msg.buttons and len(msg.buttons) > self.mode_btn:
            curr_btn = msg.buttons[self.mode_btn]
            if curr_btn == 1 and self.prev_mode_btn == 0:
                self.drive_mode = not self.drive_mode
                mode_str = 'DRIVE' if self.drive_mode else 'ARM'
                self.get_logger().info(f'Mode switched to {mode_str}')
            self.prev_mode_btn = curr_btn

        if self.drive_mode:
            self._handle_drive(msg)
        else:
            self._handle_arm(msg)

    def _handle_drive(self, msg: Joy):
        """Map joystick to Twist and publish /cmd_vel."""
        twist = Twist()

        if msg.axes and len(msg.axes) > max(self.lin_axis, self.ang_axis):
            twist.linear.x = self._apply_deadzone(
                msg.axes[self.lin_axis]) * self.lin_scale
            twist.angular.z = self._apply_deadzone(
                msg.axes[self.ang_axis]) * self.ang_scale

        self.pub_cmd_vel.publish(twist)

    def _handle_arm(self, msg: Joy):
        """Map joystick axes to arm joint velocities.

        Default mapping (6 axes for 6 joints):
          axis 0 → base_yaw
          axis 1 → shoulder_pitch
          axis 2 → elbow_pitch
          axis 3 → wrist_roll
          axis 4 → wrist_pitch
          axis 5 → gripper

        If fewer axes are available, unmapped joints get 0 velocity.
        """
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = self.ARM_JOINT_NAMES

        velocities = []
        for i in range(6):
            if msg.axes and i < len(msg.axes):
                vel = self._apply_deadzone(msg.axes[i]) * self.arm_scale
            else:
                vel = 0.0
            velocities.append(vel)

        joint_msg.velocity = velocities
        joint_msg.position = []  # Velocity-only control
        joint_msg.effort = []

        self.pub_arm.publish(joint_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
