#!/usr/bin/env python3
"""
Autonomy Node — waypoint navigation for competition mapping task. (STUB)

This is a minimal stub. The navigation logic is not yet implemented.
All services respond successfully but log "not implemented" messages.

TOPICS:
  Sub: /gps/fix (sensor_msgs/NavSatFix) — GPS position (future)
  Sub: /imu/data (sensor_msgs/Imu) — IMU orientation (future)
  Sub: /estop/active (std_msgs/Bool) — halt when true
  Pub: /cmd_vel (geometry_msgs/Twist) — drive commands
  Pub: /autonomy/status (std_msgs/String) — JSON status

SERVICES:
  /autonomy/set_waypoints (std_srvs/Trigger)
  /autonomy/start (std_srvs/Trigger)
  /autonomy/stop (std_srvs/Trigger)

HARDWARE UPGRADE:
  1. Add GPS driver node that publishes /gps/fix
  2. Add IMU driver node that publishes /imu/data
  3. Implement waypoint tracking logic in _navigate() method
  4. Load waypoints from CSV file via waypoint_file parameter
"""

import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class AutonomyNode(Node):
    """Waypoint navigation stub."""

    def __init__(self):
        super().__init__('autonomy_node')

        # -- Parameters --
        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('arrival_radius_m', 2.0)
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('heading_kp', 1.0)

        # -- State --
        self.navigating = False
        self.estop_active = False
        self.waypoints = []
        self.current_waypoint = 0
        self.gps_fix = None
        self.imu_data = None

        # -- Subscribers --
        self.create_subscription(NavSatFix, '/gps/fix', self._on_gps, 10)
        self.create_subscription(Imu, '/imu/data', self._on_imu, 10)
        self.create_subscription(Bool, '/estop/active', self._on_estop, 10)

        # -- Publishers --
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/autonomy/status', 10)

        # -- Services --
        self.create_service(Trigger, '/autonomy/set_waypoints', self._on_set_waypoints)
        self.create_service(Trigger, '/autonomy/start', self._on_start)
        self.create_service(Trigger, '/autonomy/stop', self._on_stop)

        # -- Timer --
        self.create_timer(0.1, self._navigate)    # 10 Hz
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info('Autonomy node ready (STUB — navigation not implemented)')

    def _on_gps(self, msg): self.gps_fix = msg
    def _on_imu(self, msg): self.imu_data = msg
    def _on_estop(self, msg):
        self.estop_active = msg.data
        if msg.data and self.navigating:
            self.navigating = False
            self.get_logger().warn('Navigation halted by e-stop')

    def _on_set_waypoints(self, request, response):
        # TODO: Parse waypoints from request or file
        response.success = True
        response.message = 'Waypoint loading not yet implemented'
        self.get_logger().info('set_waypoints called (stub)')
        return response

    def _on_start(self, request, response):
        if not self.waypoints:
            response.success = False
            response.message = 'No waypoints loaded'
        else:
            self.navigating = True
            self.current_waypoint = 0
            response.success = True
            response.message = 'Navigation started'
        self.get_logger().info('start called (stub)')
        return response

    def _on_stop(self, request, response):
        self.navigating = False
        self.pub_cmd_vel.publish(Twist())  # Zero velocity
        response.success = True
        response.message = 'Navigation stopped'
        self.get_logger().info('stop called')
        return response

    def _navigate(self):
        """Main navigation loop — TODO: implement waypoint tracking."""
        if not self.navigating or self.estop_active:
            return
        # TODO: Implement GPS waypoint navigation
        # 1. Get current position from /gps/fix
        # 2. Compute heading to next waypoint
        # 3. Get current heading from /imu/data
        # 4. PD controller for heading error → angular.z
        # 5. Set linear.x based on distance
        # 6. Check arrival radius, advance waypoint

    def _publish_status(self):
        status = {
            'state': 'navigating' if self.navigating else 'idle',
            'waypoint': self.current_waypoint,
            'total': len(self.waypoints),
            'gps_available': self.gps_fix is not None,
            'imu_available': self.imu_data is not None,
        }
        msg = String()
        msg.data = json.dumps(status)
        self.pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
