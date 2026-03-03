#!/usr/bin/env python3
"""
System Monitor Node — publishes power, network, and node list telemetry.

This is the main telemetry provider for the GUI Status Dashboard.
All JSON formats must match exactly what the GUI expects.

GUI INTERFACE CONTRACT:
  /system/power   — sensor_msgs/BatteryState with voltage, current, percentage
  /system/network — std_msgs/String with JSON: {"ssid":"...","signal_strength":75,"latency":12.5}
  /system/nodes   — std_msgs/String with JSON array: [{"id":"camera_node","name":"Camera Node","status":"active"},...]

  The "id" field in nodes MUST match GUI DEFAULT_NODES ids:
    camera_node, motor_ctrl, can_bridge, rs485_bridge, sensor_hub, system_monitor

MOCK MODE (Orange Pi):
  Battery: Simulates 6S LiPo with small random fluctuations
  Network: Reads REAL wifi stats (ssid, signal, latency) — works on Orange Pi
  Nodes:   Reads REAL running ROS nodes via rclpy graph API

HARDWARE UPGRADE (Jetson/RPi with battery sensor):
  1. Set use_mock=false
  2. Implement battery reading from ADC or INA226 I2C sensor
  3. See _read_real_battery() method for guidance
"""

import json
import re
import subprocess
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String


class SystemMonitorNode(Node):
    """Publishes system telemetry for the GUI dashboard."""

    # Maps ROS node names to GUI DEFAULT_NODES ids
    # The GUI expects these exact id values in the /system/nodes JSON
    NODE_ID_MAP = {
        'camera_node':         {'id': 'camera_node',    'name': 'Camera Node'},
        'drive_node':          {'id': 'motor_ctrl',     'name': 'Motor Controller'},
        'can_bridge_node':     {'id': 'can_bridge',     'name': 'CAN Bridge'},
        'rs485_bridge_node':   {'id': 'rs485_bridge',   'name': 'RS485 Bridge'},
        'system_monitor_node': {'id': 'system_monitor', 'name': 'System Monitor'},
        # sensor_hub maps to system_monitor in GUI (legacy naming)
    }

    # All GUI-expected node ids (from constants.js DEFAULT_NODES)
    EXPECTED_IDS = [
        'camera_node', 'motor_ctrl', 'can_bridge',
        'rs485_bridge', 'sensor_hub', 'system_monitor'
    ]

    def __init__(self):
        super().__init__('system_monitor_node')

        # -- Parameters --
        self.declare_parameter('use_mock', True)
        self.declare_parameter('power_topic_rate_hz', 1.0)
        self.declare_parameter('network_topic_rate_hz', 2.0)
        self.declare_parameter('node_list_rate_hz', 1.0)
        self.declare_parameter('battery_cells', 6)
        self.declare_parameter('cell_voltage_min', 3.3)
        self.declare_parameter('cell_voltage_max', 4.2)
        self.declare_parameter('ping_host', '192.168.1.1')

        self.use_mock = self.get_parameter('use_mock').value

        # -- Publishers --
        self.pub_power = self.create_publisher(BatteryState, '/system/power', 10)
        self.pub_network = self.create_publisher(String, '/system/network', 10)
        self.pub_nodes = self.create_publisher(String, '/system/nodes', 10)

        # -- Timers --
        self.create_timer(
            1.0 / self.get_parameter('power_topic_rate_hz').value,
            self._publish_power)
        self.create_timer(
            1.0 / self.get_parameter('network_topic_rate_hz').value,
            self._publish_network)
        self.create_timer(
            1.0 / self.get_parameter('node_list_rate_hz').value,
            self._publish_nodes)

        mode = 'MOCK battery' if self.use_mock else 'REAL battery sensor'
        self.get_logger().info(f'System monitor started ({mode}, real network/nodes)')

    # ================================================================== #
    # Power
    # ================================================================== #

    def _publish_power(self):
        """Publish battery state."""
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()

        if self.use_mock:
            # Simulate 6S LiPo at ~75% charge
            msg.voltage = 23.5 + random.uniform(-0.3, 0.3)
            msg.current = 4.5 + random.uniform(-1.0, 1.0)
            msg.percentage = self._voltage_to_percentage(msg.voltage)
            msg.present = True
        else:
            self._read_real_battery(msg)

        self.pub_power.publish(msg)

    def _voltage_to_percentage(self, total_voltage):
        """Convert pack voltage to percentage based on cell count."""
        cells = self.get_parameter('battery_cells').value
        v_min = self.get_parameter('cell_voltage_min').value
        v_max = self.get_parameter('cell_voltage_max').value

        cell_v = total_voltage / cells
        pct = (cell_v - v_min) / (v_max - v_min)
        return max(0.0, min(1.0, pct))

    def _read_real_battery(self, msg):
        """Read battery from hardware sensor.

        HARDWARE UPGRADE: Implement one of these approaches:

        Option A — INA226 I2C sensor:
          import smbus2
          bus = smbus2.SMBus(1)
          raw = bus.read_word_data(0x40, 0x02)  # Bus voltage register
          msg.voltage = raw * 1.25e-3  # LSB = 1.25 mV
          raw_current = bus.read_word_data(0x40, 0x01)
          msg.current = raw_current * 0.001  # Depends on shunt resistor

        Option B — ADC (e.g., ADS1115):
          import board, busio, adafruit_ads1x15.ads1115 as ADS
          from adafruit_ads1x15.analog_in import AnalogIn
          i2c = busio.I2C(board.SCL, board.SDA)
          ads = ADS.ADS1115(i2c)
          chan = AnalogIn(ads, ADS.P0)
          msg.voltage = chan.voltage * VOLTAGE_DIVIDER_RATIO
        """
        msg.voltage = 0.0
        msg.current = 0.0
        msg.percentage = 0.0
        msg.present = False

    # ================================================================== #
    # Network
    # ================================================================== #

    def _publish_network(self):
        """Publish network status as JSON string.

        JSON format: {"ssid": "...", "signal_strength": 75, "latency": 12.5}
        This format matches what SystemStatusContext.jsx parses.
        """
        data = {
            'ssid': self._get_ssid(),
            'signal_strength': self._get_signal_strength(),
            'latency': self._get_latency(),
        }

        msg = String()
        msg.data = json.dumps(data)
        self.pub_network.publish(msg)

    def _get_ssid(self):
        """Get current WiFi SSID."""
        try:
            result = subprocess.run(
                ['iwgetid', '-r'], capture_output=True, text=True, timeout=2)
            if result.returncode == 0 and result.stdout.strip():
                return result.stdout.strip()
        except FileNotFoundError:
            pass
        except Exception:
            pass

        # Fallback: nmcli
        try:
            result = subprocess.run(
                ['nmcli', '-t', '-f', 'active,ssid', 'dev', 'wifi'],
                capture_output=True, text=True, timeout=2)
            for line in result.stdout.strip().split('\n'):
                if line.startswith('yes:'):
                    return line.split(':', 1)[1]
        except Exception:
            pass

        return 'Unknown'

    def _get_signal_strength(self):
        """Get WiFi signal strength as integer percentage (0-100)."""
        try:
            result = subprocess.run(
                ['iwconfig'], capture_output=True, text=True, timeout=2)
            match = re.search(r'Signal level[=:](-?\d+)', result.stdout)
            if match:
                dbm = int(match.group(1))
                # Map -90 dBm → 0%, -30 dBm → 100%
                return max(0, min(100, (dbm + 90) * 100 // 60))

            # Some drivers report as x/100
            match = re.search(r'Signal level[=:](\d+)/(\d+)', result.stdout)
            if match:
                return int(int(match.group(1)) / int(match.group(2)) * 100)
        except Exception:
            pass
        return 0

    def _get_latency(self):
        """Get ping latency to base station in milliseconds."""
        try:
            host = self.get_parameter('ping_host').value
            result = subprocess.run(
                ['ping', '-c', '1', '-W', '1', host],
                capture_output=True, text=True, timeout=3)
            match = re.search(r'time=(\d+\.?\d*)', result.stdout)
            if match:
                return round(float(match.group(1)), 1)
        except Exception:
            pass
        return 0.0

    # ================================================================== #
    # Node List
    # ================================================================== #

    def _publish_nodes(self):
        """Publish active ROS node list as JSON array.

        JSON format: [{"id": "camera_node", "name": "Camera Node", "status": "active"}, ...]
        The "id" values MUST match GUI DEFAULT_NODES ids from constants.js.
        """
        # Get running node names via rclpy graph API
        try:
            running_names = set()
            for name, namespace in self.get_node_names_and_namespaces():
                # Strip namespace prefix, get bare name
                running_names.add(name)
        except Exception:
            running_names = set()

        # Build JSON array matching GUI expectations
        nodes = []

        # Map running ROS nodes to GUI ids
        active_gui_ids = set()
        for ros_name, gui_info in self.NODE_ID_MAP.items():
            is_active = ros_name in running_names
            if is_active:
                active_gui_ids.add(gui_info['id'])
            nodes.append({
                'id': gui_info['id'],
                'name': gui_info['name'],
                'status': 'active' if is_active else 'inactive',
            })

        # Add any expected GUI ids not yet in the list
        existing_ids = {n['id'] for n in nodes}
        for gui_id in self.EXPECTED_IDS:
            if gui_id not in existing_ids:
                # sensor_hub is a legacy duplicate of system_monitor
                name_map = {
                    'camera_node': 'Camera Node',
                    'motor_ctrl': 'Motor Controller',
                    'can_bridge': 'CAN Bridge',
                    'rs485_bridge': 'RS485 Bridge',
                    'sensor_hub': 'Sensor Hub',
                    'system_monitor': 'System Monitor',
                }
                nodes.append({
                    'id': gui_id,
                    'name': name_map.get(gui_id, gui_id),
                    'status': 'active' if gui_id in active_gui_ids else 'inactive',
                })

        msg = String()
        msg.data = json.dumps(nodes)
        self.pub_nodes.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
