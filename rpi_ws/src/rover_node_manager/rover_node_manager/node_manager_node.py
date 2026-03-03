#!/usr/bin/env python3
"""
Node Manager Node — provides services for the GUI to launch/stop ROS nodes.

Uses custom rover_interfaces/srv/NodeControl service which has:
  Request:  string data (JSON payload)
  Response: bool success, string message

GUI sends:
  Launch: {"node_id": "camera_node", "package": "rover_camera", "executable": "camera_node"}
  Stop:   {"node_id": "camera_node"}

This node manages subprocesses via `ros2 run <package> <executable>`.

SERVICES:
  /node_manager/launch (rover_interfaces/srv/NodeControl) — Launch a node
  /node_manager/stop   (rover_interfaces/srv/NodeControl) — Stop a node
  /node_manager/list   (std_srvs/srv/Trigger)             — List managed nodes

MOCK MODE: Fully functional on Orange Pi (launches real ROS nodes).
HARDWARE UPGRADE: No changes needed.
"""

import json
import os
import signal
import subprocess

import rclpy
from rclpy.node import Node
from rover_interfaces.srv import NodeControl
from std_srvs.srv import Trigger


class NodeManagerNode(Node):
    """Manages ROS node subprocesses for GUI control."""

    def __init__(self):
        super().__init__('node_manager_node')

        # Map: node_id -> {'process': Popen, 'package': str, 'executable': str}
        self.managed = {}

        # -- Services --
        self.create_service(
            NodeControl, '/node_manager/launch', self._on_launch)
        self.create_service(
            NodeControl, '/node_manager/stop', self._on_stop)
        self.create_service(
            Trigger, '/node_manager/list', self._on_list)

        # Periodic check for crashed processes
        self.create_timer(2.0, self._check_processes)

        self.get_logger().info('Node manager ready — waiting for GUI commands')

    # ------------------------------------------------------------------ #
    # Service Handlers
    # ------------------------------------------------------------------ #

    def _on_launch(self, request, response):
        """Launch a node. Request data is JSON with node_id, package, executable."""
        try:
            data = json.loads(request.data)
            node_id = data.get('node_id', '')
            package = data.get('package', '')
            executable = data.get('executable', '')

            if not all([node_id, package, executable]):
                response.success = False
                response.message = 'Missing required fields: node_id, package, executable'
                return response

            # Check if already running
            if node_id in self.managed:
                proc = self.managed[node_id]['process']
                if proc.poll() is None:
                    response.success = False
                    response.message = f'{node_id} is already running (PID {proc.pid})'
                    return response
                else:
                    # Previous process exited, clean up
                    del self.managed[node_id]

            # Launch via ros2 run
            cmd = ['ros2', 'run', package, executable]
            self.get_logger().info(f'Launching: {" ".join(cmd)}')

            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,  # New process group for clean kill
            )

            self.managed[node_id] = {
                'process': proc,
                'package': package,
                'executable': executable,
            }

            response.success = True
            response.message = f'Launched {node_id} (PID {proc.pid})'
            self.get_logger().info(response.message)

        except json.JSONDecodeError as e:
            response.success = False
            response.message = f'Invalid JSON: {e}'
        except Exception as e:
            response.success = False
            response.message = f'Launch failed: {e}'
            self.get_logger().error(response.message)

        return response

    def _on_stop(self, request, response):
        """Stop a managed node. Request data is JSON with node_id."""
        try:
            data = json.loads(request.data)
            node_id = data.get('node_id', '')

            if not node_id:
                response.success = False
                response.message = 'Missing node_id'
                return response

            if node_id not in self.managed:
                response.success = False
                response.message = f'{node_id} is not managed by this node'
                return response

            proc = self.managed[node_id]['process']

            if proc.poll() is not None:
                del self.managed[node_id]
                response.success = True
                response.message = f'{node_id} already stopped'
                return response

            # Graceful shutdown: SIGINT to process group
            self.get_logger().info(f'Stopping {node_id} (PID {proc.pid})')
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                proc.wait(timeout=5.0)
                response.message = f'Stopped {node_id}'
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                proc.wait(timeout=2.0)
                response.message = f'Force-killed {node_id}'
            except ProcessLookupError:
                response.message = f'{node_id} already exited'

            del self.managed[node_id]
            response.success = True
            self.get_logger().info(response.message)

        except json.JSONDecodeError as e:
            response.success = False
            response.message = f'Invalid JSON: {e}'
        except Exception as e:
            response.success = False
            response.message = f'Stop failed: {e}'
            self.get_logger().error(response.message)

        return response

    def _on_list(self, request, response):
        """List all managed nodes and their status."""
        node_list = []
        for node_id, info in self.managed.items():
            proc = info['process']
            status = 'running' if proc.poll() is None else 'stopped'
            node_list.append({
                'id': node_id,
                'status': status,
                'pid': proc.pid,
                'package': info['package'],
                'executable': info['executable'],
            })

        response.success = True
        response.message = json.dumps(node_list)
        return response

    # ------------------------------------------------------------------ #
    # Process Monitor
    # ------------------------------------------------------------------ #

    def _check_processes(self):
        """Check for crashed processes and log warnings."""
        for node_id, info in list(self.managed.items()):
            proc = info['process']
            if proc.poll() is not None:
                exit_code = proc.returncode
                if exit_code != 0:
                    self.get_logger().warn(
                        f'{node_id} crashed (exit code {exit_code})')
                else:
                    self.get_logger().info(f'{node_id} exited cleanly')

    def _cleanup_all(self):
        """Stop all managed processes on shutdown."""
        for node_id, info in self.managed.items():
            proc = info['process']
            if proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    proc.wait(timeout=3.0)
                except Exception:
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    except Exception:
                        pass


def main(args=None):
    rclpy.init(args=args)
    node = NodeManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cleanup_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
