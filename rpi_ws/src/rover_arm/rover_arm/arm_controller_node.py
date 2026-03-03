#!/usr/bin/env python3
"""
6-DOF Arm Controller Node — applies safety limits to joint commands.

Sits between joystick teleop (/arm/target_joints) and the CAN bridge
(/arm/joint_commands). Clamps positions and velocities to safe limits.

Joint names (from arm_motors.yaml):
  base_yaw, shoulder_pitch, elbow_pitch, wrist_roll, wrist_pitch, gripper

TOPICS:
  Sub: /arm/target_joints (sensor_msgs/JointState) — desired from teleop
  Sub: /arm/joint_states  (sensor_msgs/JointState) — feedback from CAN
  Sub: /estop/active      (std_msgs/Bool)           — suppress when true
  Pub: /arm/joint_commands (sensor_msgs/JointState) — clamped → CAN bridge
  Pub: /arm/status         (std_msgs/String)         — JSON status

PARAMETERS:
  velocity_scale (double, default=0.5) — scale for velocity commands (0-1)
  publish_rate_hz (double, default=50.0) — command publish rate

MOCK MODE: No hardware dependencies. Works on Orange Pi.
HARDWARE UPGRADE: No changes needed — adjust limits in arm_motors.yaml.
"""

import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String


class ArmControllerNode(Node):
    """Arm controller with safety limits."""

    JOINT_NAMES = [
        'base_yaw', 'shoulder_pitch', 'elbow_pitch',
        'wrist_roll', 'wrist_pitch', 'gripper'
    ]

    # Default safety limits (from ros_architecture.md motor config)
    # HARDWARE UPGRADE: Load these from arm_motors.yaml via parameter
    POSITION_LIMITS = {
        'base_yaw':       (-3.14, 3.14),
        'shoulder_pitch': (-1.57, 1.57),
        'elbow_pitch':    (-2.35, 2.35),
        'wrist_roll':     (-3.14, 3.14),
        'wrist_pitch':    (-1.57, 1.57),
        'gripper':        (0.0,   1.57),
    }

    VELOCITY_LIMITS = {
        'base_yaw':       10.0,
        'shoulder_pitch': 5.0,
        'elbow_pitch':    5.0,
        'wrist_roll':     8.0,
        'wrist_pitch':    8.0,
        'gripper':        10.0,
    }

    def __init__(self):
        super().__init__('arm_controller_node')

        # -- Parameters --
        self.declare_parameter('velocity_scale', 0.5)
        self.declare_parameter('publish_rate_hz', 50.0)

        self.vel_scale = self.get_parameter('velocity_scale').value

        # -- State --
        self.estop_active = False
        self.target = None           # Latest target JointState
        self.feedback_positions = [0.0] * 6

        # -- Subscribers --
        self.create_subscription(
            JointState, '/arm/target_joints', self._on_target, 10)
        self.create_subscription(
            JointState, '/arm/joint_states', self._on_feedback, 10)
        self.create_subscription(
            Bool, '/estop/active', self._on_estop, 10)

        # -- Publishers --
        self.pub_commands = self.create_publisher(
            JointState, '/arm/joint_commands', 10)
        self.pub_status = self.create_publisher(String, '/arm/status', 10)

        # -- Timers --
        rate = self.get_parameter('publish_rate_hz').value
        self.create_timer(1.0 / rate, self._control_loop)
        self.create_timer(0.2, self._publish_status)  # 5 Hz status

        self.get_logger().info(
            f'Arm controller ready (vel_scale={self.vel_scale})')

    def _on_target(self, msg: JointState):
        self.target = msg

    def _on_feedback(self, msg: JointState):
        if msg.position:
            self.feedback_positions = list(msg.position)[:6]

    def _on_estop(self, msg: Bool):
        self.estop_active = msg.data

    def _control_loop(self):
        """Apply safety limits and publish commands."""
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = self.JOINT_NAMES

        if self.estop_active or self.target is None:
            # E-stop or no target: zero velocities
            cmd.position = list(self.feedback_positions)
            cmd.velocity = [0.0] * 6
            cmd.effort = [0.0] * 6
        else:
            # Clamp target positions and velocities
            positions = list(self.target.position) if self.target.position else [0.0] * 6
            velocities = list(self.target.velocity) if self.target.velocity else [0.0] * 6

            # Pad to 6
            positions += [0.0] * (6 - len(positions))
            velocities += [0.0] * (6 - len(velocities))

            clamped_pos = []
            clamped_vel = []

            for i, name in enumerate(self.JOINT_NAMES):
                # Clamp position
                pmin, pmax = self.POSITION_LIMITS[name]
                clamped_pos.append(max(pmin, min(pmax, positions[i])))

                # Clamp and scale velocity
                vmax = self.VELOCITY_LIMITS[name] * self.vel_scale
                clamped_vel.append(max(-vmax, min(vmax, velocities[i])))

            cmd.position = clamped_pos
            cmd.velocity = clamped_vel
            cmd.effort = [0.0] * 6

        self.pub_commands.publish(cmd)

    def _publish_status(self):
        """Publish arm status as JSON."""
        status = {
            'estop': self.estop_active,
            'has_target': self.target is not None,
            'positions': self.feedback_positions,
        }
        msg = String()
        msg.data = json.dumps(status)
        self.pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
