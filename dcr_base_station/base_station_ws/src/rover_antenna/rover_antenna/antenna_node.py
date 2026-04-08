import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class AntennaNode(Node):

    def __init__(self):
        super().__init__('antenna_node')

        self.declare_parameter('serial_port', '/dev/ttyACM1')
        self.declare_parameter('baud_rate', 115200)

        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value

        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=1)
            self.get_logger().info(f'Opened serial port {serial_port} at {baud_rate} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {serial_port}: {e}')
            self.ser = None

        self.subscription = self.create_subscription(
            String,
            '/antenna/trigger',
            self.trigger_callback,
            10
        )

    def trigger_callback(self, msg):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn('Serial port not open, cannot send command')
            return
        if not msg.data:
            return
        self.ser.write(msg.data.encode())
        self.get_logger().info(f'Sent "{msg.data}" to antenna ESP32')

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AntennaNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
