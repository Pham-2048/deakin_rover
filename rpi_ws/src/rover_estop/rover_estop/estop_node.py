#!/usr/bin/env python3
"""
Emergency Stop Coordinator Node.

Highest-priority safety node. When triggered (via /emergency_stop from GUI or
watchdog timeout), latches e-stop state and commands all actuators to zero.

TOPICS:
  Sub: /emergency_stop (std_msgs/Bool) - GUI publishes {data: true} to trigger
  Pub: /estop/active (std_msgs/Bool) - Current latched e-stop state
  Pub: /cmd_vel (geometry_msgs/Twist) - Publishes zero twist while active
  Pub: /arm/joint_commands (sensor_msgs/JointState) - Publishes zero velocities while active

SERVICES:
  /estop/reset (std_srvs/Trigger) - Manually reset the e-stop latch

PARAMETERS:
  watchdog_timeout_sec (double, default=2.0) - Auto-trigger if no /emergency_stop msg received
  require_manual_reset (bool, default=true) - If true, must call /estop/reset to clear

MOCK MODE: No hardware dependencies. Fully functional on Orange Pi.
HARDWARE UPGRADE: No changes needed.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


class EStopNode(Node):
    """Emergency stop coordinator - overrides all motion commands when active."""

    # Arm joint names matching config/arm_motors.yaml
    ARM_JOINT_NAMES = [
        'base_yaw', 'shoulder_pitch', 'elbow_pitch',
        'wrist_roll', 'wrist_pitch', 'gripper'
    ]

    def __init__(self):
        super().__init__('estop_node')

        # -- Parameters --
        self.declare_parameter('watchdog_timeout_sec', 2.0)
        self.declare_parameter('require_manual_reset', True)

        self.watchdog_timeout = self.get_parameter('watchdog_timeout_sec').value
        self.require_manual_reset = self.get_parameter('require_manual_reset').value

        # -- State --
        self.estop_active = False
        self.last_heartbeat = self.get_clock().now()

        # -- Subscribers --
        self.create_subscription(Bool, '/emergency_stop', self._on_emergency, 10)

        # -- Publishers --
        self.pub_active = self.create_publisher(Bool, '/estop/active', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_arm_cmd = self.create_publisher(JointState, '/arm/joint_commands', 10)

        # -- Services --
        self.create_service(Trigger, '/estop/reset', self._on_reset)

        # -- Timers --
        self.create_timer(0.02, self._control_loop)    # 50 Hz
        self.create_timer(0.5, self._watchdog_check)    # 2 Hz watchdog check

        self.get_logger().info(
            f'E-Stop node ready (watchdog={self.watchdog_timeout}s, '
            f'manual_reset={self.require_manual_reset})')

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #

    def _on_emergency(self, msg: Bool):
        """Handle /emergency_stop from GUI."""
        self.last_heartbeat = self.get_clock().now()

        if msg.data:
            if not self.estop_active:
                self.estop_active = True
                self.get_logger().warn('E-STOP ACTIVATED (GUI trigger)')
        elif not self.require_manual_reset:
            if self.estop_active:
                self.estop_active = False
                self.get_logger().info('E-Stop auto-cleared (require_manual_reset=false)')

    def _on_reset(self, request, response):
        """Service handler: /estop/reset."""
        if self.estop_active:
            self.estop_active = False
            self.last_heartbeat = self.get_clock().now()
            response.success = True
            response.message = 'E-Stop reset successfully'
            self.get_logger().info('E-Stop manually reset')
        else:
            response.success = True
            response.message = 'E-Stop was already inactive'
        return response

    # ------------------------------------------------------------------ #
    # Timers
    # ------------------------------------------------------------------ #

    def _watchdog_check(self):
        """Auto-trigger e-stop if GUI heartbeat is lost."""
        elapsed = (self.get_clock().now() - self.last_heartbeat).nanoseconds / 1e9
        if elapsed > self.watchdog_timeout and not self.estop_active:
            self.estop_active = True
            self.get_logger().error(
                f'WATCHDOG TIMEOUT ({self.watchdog_timeout}s) - E-STOP AUTO-TRIGGERED. '
                f'No /emergency_stop message received.')

    def _control_loop(self):
        """50 Hz loop: publish e-stop state and zero commands when active."""
        # Always publish current state
        active_msg = Bool(data=self.estop_active)
        self.pub_active.publish(active_msg)

        # When active, override all motion with zeros
        if self.estop_active:
            self.pub_cmd_vel.publish(Twist())  # All zeros

            zero_arm = JointState()
            zero_arm.header.stamp = self.get_clock().now().to_msg()
            zero_arm.name = self.ARM_JOINT_NAMES
            zero_arm.velocity = [0.0] * len(self.ARM_JOINT_NAMES)
            self.pub_arm_cmd.publish(zero_arm)


def main(args=None):
    rclpy.init(args=args)
    node = EStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
