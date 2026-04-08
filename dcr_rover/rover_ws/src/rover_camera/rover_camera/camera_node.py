import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)

        device = self.get_parameter('device').get_parameter_value().string_value
        width = self.get_parameter('width').get_parameter_value().integer_value
        height = self.get_parameter('height').get_parameter_value().integer_value
        fps = self.get_parameter('fps').get_parameter_value().integer_value

        self._bridge = CvBridge()
        self._pub = self.create_publisher(Image, 'image_raw', 1)

        self._cap = cv2.VideoCapture(device)
        if not self._cap.isOpened():
            self.get_logger().error(f'Cannot open camera device: {device}')
        else:
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self._cap.set(cv2.CAP_PROP_FPS, fps)
            self.get_logger().info(
                f'Opened {device} at {width}x{height} @ {fps} fps'
            )

        self._timer = self.create_timer(1.0 / fps, self._publish_frame)

    def _publish_frame(self):
        if not self._cap.isOpened():
            return
        ret, frame = self._cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame', throttle_duration_sec=5.0)
            return
        try:
            msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            self._pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to publish frame: {e}', throttle_duration_sec=5.0)

    def destroy_node(self):
        if self._cap.isOpened():
            self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
