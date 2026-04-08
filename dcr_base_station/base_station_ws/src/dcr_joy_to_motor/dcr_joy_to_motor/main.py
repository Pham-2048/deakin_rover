#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from dcr_interfaces.srv import MotorMovementCommand
from dcr_interfaces.msg import ESP32LED
from sensor_msgs.msg import Joy

class Teleop(Node):

    def led_something(self):
        try:
            led_msg = ESP32LED()
            led_msg.green = 128
            led_msg.start_address = 0
            led_msg.stop_address = 211

            self.publisher.publish(led_msg)
        except:
            self.get_logger().error('Could not send LED command')

    def __init__(self):
        super().__init__("teleop") # TODO give me a better name :(

        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.listener_callback,
            10
        )

        self.publisher = self.create_publisher(ESP32LED, '/esp32/led_command', 1)

        self.client = self.create_client(MotorMovementCommand, '/motor_rs485_service/issue_motor_command')

        self.latest_message = None
        self.last_joy_time = None
        self.joy_timeout_sec = 0.5  # stop motors if no /joy message for 500ms
        self.processing_rate_hz = 3 # Process 2 messages per second

        self.create_timer(1/self.processing_rate_hz, self.process_teleop)

        while not self.client.wait_for_service(timeout_sec=1):
            self.get_logger().warn('Motor Controller Service not available, waiting again...')
        
        self.request = MotorMovementCommand.Request()

    def listener_callback(self, msg):
        """Stores the latest joystick message"""
        self.latest_message = msg
        self.last_joy_time = self.get_clock().now()
        self.led_something()

    def send_motor_request(self, address, speed, direction):
        """Sends motor movement request using the service client"""

        if not self.client.service_is_ready():
            self.get_logger().warn("Motor Controller Service is unavailable, skipping request.")
            return

        self.request.address = address
        self.request.speed = speed
        self.request.direction = direction

        future = self.client.call_async(self.request)

        return 1



    def process_teleop(self):
        try:
            # Watchdog: stop motors if /joy has not been received recently
            if self.last_joy_time is not None:
                elapsed = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
                if elapsed > self.joy_timeout_sec:
                    self.get_logger().warn('No /joy message for %.2fs - stopping motors' % elapsed)
                    self.send_motor_request(1, 0, 3)
                    self.send_motor_request(2, 0, 3)
                    self.latest_message = None
                    self.last_joy_time = None
                    return

            msg = self.latest_message

            if(msg):
                left_joystick = self.scale_joystick_output(msg.axes[1]) ## There are instances where this does not yet exist..
                right_joystick = self.scale_joystick_output(msg.axes[3]) ## therefore causing this to crash, TODO: FIX ME

                self.get_logger().info('Left joystick: "%f"' % left_joystick)
                self.get_logger().info('Right joystick: "%f"' % right_joystick)

                left_direction = 3
                right_direction = 3

                if left_joystick > 0: 
                    left_direction = 1
                if right_joystick > 0: 
                    right_direction = 2
                if left_joystick < 0: 
                    left_direction = 2
                if right_joystick < 0: 
                    right_direction = 1

                self.send_motor_request(1, abs(left_joystick), left_direction)
                self.send_motor_request(2, abs(right_joystick), right_direction)
                print(right_joystick)
                self.latest_message = None
        except:
            self.get_logger().error('Something went wrong')

    def scale_joystick_output(self, value, scale=800, deadzone=0.08):
        if abs(value) < deadzone:
            return int(0) 
        
        return int(value * scale)


def main(args=None):
    rclpy.init(args=args)
    teleop = Teleop()
    
    rclpy.spin(teleop)

    teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
