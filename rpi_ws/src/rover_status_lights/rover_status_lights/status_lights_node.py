#!/usr/bin/env python3
"""
Status Lights Node — competition LED indicators via GPIO. (STUB)

Controls 3 LEDs for competition requirements:
  RED   = E-stop active
  GREEN = System ready
  BLUE  = Autonomous mode

MOCK MODE (Orange Pi / no GPIO LEDs):
  Logs LED state changes to console instead of toggling GPIO pins.

HARDWARE UPGRADE (Jetson/RPi with LEDs wired to GPIO):
  1. Set use_mock=false
  2. Set gpio_red, gpio_green, gpio_blue to your pin numbers
  3. Install RPi.GPIO or Jetson.GPIO:
       pip3 install RPi.GPIO     (Raspberry Pi)
       pip3 install Jetson.GPIO  (Jetson)
  4. Wire LEDs with appropriate resistors to the GPIO pins

TOPICS:
  Sub: /estop/active (std_msgs/Bool) — red LED
  Sub: /autonomy/status (std_msgs/String) — blue LED

PARAMETERS:
  use_mock (bool, default=true)
  gpio_red (int, default=17)
  gpio_green (int, default=27)
  gpio_blue (int, default=22)
"""

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

try:
    import RPi.GPIO as GPIO
    HAS_GPIO = True
except ImportError:
    HAS_GPIO = False


class StatusLightsNode(Node):
    """Competition LED controller."""

    def __init__(self):
        super().__init__('status_lights_node')

        # -- Parameters --
        self.declare_parameter('use_mock', True)
        self.declare_parameter('gpio_red', 17)
        self.declare_parameter('gpio_green', 27)
        self.declare_parameter('gpio_blue', 22)

        self.use_mock = self.get_parameter('use_mock').value
        self.pin_red = self.get_parameter('gpio_red').value
        self.pin_green = self.get_parameter('gpio_green').value
        self.pin_blue = self.get_parameter('gpio_blue').value

        # -- State --
        self.estop_active = False
        self.autonomous = False

        # -- Init GPIO --
        if not self.use_mock and HAS_GPIO:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.pin_red, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.pin_green, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.pin_blue, GPIO.OUT, initial=GPIO.LOW)

        # -- Subscribers --
        self.create_subscription(Bool, '/estop/active', self._on_estop, 10)
        self.create_subscription(String, '/autonomy/status', self._on_autonomy, 10)

        # -- Timer --
        self.create_timer(0.1, self._update_leds)

        self.get_logger().info(
            f'Status lights ready ({"MOCK" if self.use_mock else "GPIO"})')

    def _on_estop(self, msg):
        self.estop_active = msg.data

    def _on_autonomy(self, msg):
        try:
            data = json.loads(msg.data)
            self.autonomous = data.get('state') == 'navigating'
        except Exception:
            self.autonomous = False

    def _update_leds(self):
        red = self.estop_active
        green = not self.estop_active
        blue = self.autonomous and not self.estop_active

        if self.use_mock:
            return  # Silent in mock — only log on change
        elif HAS_GPIO:
            GPIO.output(self.pin_red, GPIO.HIGH if red else GPIO.LOW)
            GPIO.output(self.pin_green, GPIO.HIGH if green else GPIO.LOW)
            GPIO.output(self.pin_blue, GPIO.HIGH if blue else GPIO.LOW)

    def destroy_node(self):
        if not self.use_mock and HAS_GPIO:
            GPIO.output(self.pin_red, GPIO.LOW)
            GPIO.output(self.pin_green, GPIO.LOW)
            GPIO.output(self.pin_blue, GPIO.LOW)
            GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StatusLightsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
