#!/usr/bin/env python3
"""
Camera Publisher Node — publishes 3 USB camera feeds as ROS Image topics.

The GUI does NOT subscribe to these via rosbridge. Instead, web_video_server
reads these topics and serves MJPEG streams over HTTP on port 8080.
GUI fetches: http://{roverIP}:8080/stream?topic=/camera1/image_raw&type=mjpeg&quality=80

MOCK MODE (Orange Pi / no cameras):
  Publishes color-coded test pattern images with a rolling timestamp.
  - camera1 = Blue   (Front Camera)
  - camera2 = Green  (Rear Camera)
  - camera3 = Red    (Arm Camera)

HARDWARE UPGRADE (Jetson/RPi with real cameras):
  1. Set parameter use_mock=false
  2. Set camera_devices to the correct /dev/videoN paths
  3. Verify with: v4l2-ctl --list-devices

PARAMETERS:
  use_mock (bool, default=true) — Publish test patterns instead of real video
  camera_devices (string[], default=["/dev/video0","/dev/video2","/dev/video4"])
  camera_names (string[], default=["camera1","camera2","camera3"])
  frame_rate (double, default=15.0) — Target FPS per camera
  resolution_width (int, default=640)
  resolution_height (int, default=480)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import time
import math

# cv_bridge may not be available on all systems — graceful fallback
try:
    from cv_bridge import CvBridge
    HAS_CV_BRIDGE = True
except ImportError:
    HAS_CV_BRIDGE = False

try:
    import cv2
    HAS_OPENCV = True
except ImportError:
    HAS_OPENCV = False


class CameraNode(Node):
    """Publishes camera images from USB cameras or test patterns."""

    # Color map for mock cameras (BGR for OpenCV)
    MOCK_COLORS = {
        'camera1': (255, 100, 50),    # Blue-ish  (Front)
        'camera2': (50, 200, 50),     # Green     (Rear)
        'camera3': (50, 50, 255),     # Red       (Arm)
    }

    def __init__(self):
        super().__init__('camera_node')

        # -- Parameters --
        self.declare_parameter('use_mock', True)
        self.declare_parameter('camera_devices',
                               ['/dev/video0', '/dev/video2', '/dev/video4'])
        self.declare_parameter('camera_names',
                               ['camera1', 'camera2', 'camera3'])
        self.declare_parameter('frame_rate', 15.0)
        self.declare_parameter('resolution_width', 640)
        self.declare_parameter('resolution_height', 480)

        self.use_mock = self.get_parameter('use_mock').value
        self.devices = self.get_parameter('camera_devices').value
        self.names = self.get_parameter('camera_names').value
        self.fps = self.get_parameter('frame_rate').value
        self.width = self.get_parameter('resolution_width').value
        self.height = self.get_parameter('resolution_height').value

        # cv_bridge for real cameras
        self.bridge = CvBridge() if HAS_CV_BRIDGE else None

        # Publishers and camera handles
        self.img_pubs = []
        self.info_pubs = []
        self.captures = []  # cv2.VideoCapture handles (real mode) or None
        self.frame_count = 0

        for i, name in enumerate(self.names):
            self.img_pubs.append(
                self.create_publisher(Image, f'/{name}/image_raw', 10))
            self.info_pubs.append(
                self.create_publisher(CameraInfo, f'/{name}/camera_info', 10))

            if self.use_mock:
                self.captures.append(None)
            else:
                self._open_camera(i, name)

        # Timer
        self.create_timer(1.0 / self.fps, self._publish_frames)

        mode_str = 'MOCK (test patterns)' if self.use_mock else 'REAL (USB cameras)'
        self.get_logger().info(f'Camera node started — {mode_str}, {len(self.names)} cameras @ {self.fps} FPS')

    def _open_camera(self, index, name):
        """Open a real USB camera via OpenCV."""
        if not HAS_OPENCV:
            self.get_logger().error(f'OpenCV not available — cannot open {name}')
            self.captures.append(None)
            return

        device = self.devices[index] if index < len(self.devices) else f'/dev/video{index * 2}'
        cap = cv2.VideoCapture(device)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FPS, self.fps)
            self.captures.append(cap)
            self.get_logger().info(f'Opened {device} for {name}')
        else:
            self.captures.append(None)
            self.get_logger().warn(
                f'Failed to open {device} for {name} — skipping. '
                f'Check device path with: v4l2-ctl --list-devices')

    def _publish_frames(self):
        """Publish one frame per camera."""
        self.frame_count += 1
        now = self.get_clock().now().to_msg()

        for i, (name, img_pub, info_pub) in enumerate(
                zip(self.names, self.img_pubs, self.info_pubs)):

            if self.use_mock:
                img_msg = self._create_mock_frame(name, now)
            else:
                img_msg = self._capture_real_frame(i, name, now)

            if img_msg is not None:
                img_pub.publish(img_msg)

            # Always publish CameraInfo (empty calibration, can be filled later)
            info = CameraInfo()
            info.header.stamp = now
            info.header.frame_id = f'{name}_optical_frame'
            info.width = self.width
            info.height = self.height
            info_pub.publish(info)

    def _create_mock_frame(self, name, stamp):
        """Generate a color-coded test pattern with timestamp overlay."""
        color = self.MOCK_COLORS.get(name, (128, 128, 128))
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # Background color with subtle gradient
        for row in range(self.height):
            factor = 0.6 + 0.4 * (row / self.height)
            img[row, :] = [int(c * factor) for c in color]

        # Animated element — a moving bar
        bar_y = int((math.sin(self.frame_count * 0.05) + 1) / 2 * (self.height - 40))
        img[bar_y:bar_y + 4, :] = [255, 255, 255]

        if HAS_OPENCV:
            # Text overlays
            label = name.replace('camera', 'CAM ')
            cv2.putText(img, label, (20, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)
            cv2.putText(img, 'MOCK MODE', (20, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
            ts = f'{stamp.sec}.{stamp.nanosec // 1000000:03d}'
            cv2.putText(img, ts, (20, self.height - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            cv2.putText(img, f'Frame: {self.frame_count}', (self.width - 200, self.height - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # Convert to ROS Image (manual — no cv_bridge dependency for mock)
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = f'{name}_optical_frame'
        msg.height = self.height
        msg.width = self.width
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = self.width * 3
        msg.data = img.tobytes()
        return msg

    def _capture_real_frame(self, index, name, stamp):
        """Capture frame from real USB camera."""
        cap = self.captures[index] if index < len(self.captures) else None
        if cap is None or not cap.isOpened():
            return None

        ret, frame = cap.read()
        if not ret:
            self.get_logger().debug(f'Frame capture failed for {name}')
            return None

        if self.bridge:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        else:
            # Manual conversion fallback
            msg = Image()
            msg.height = frame.shape[0]
            msg.width = frame.shape[1]
            msg.encoding = 'bgr8'
            msg.is_bigendian = False
            msg.step = frame.shape[1] * 3
            msg.data = frame.tobytes()

        msg.header.stamp = stamp
        msg.header.frame_id = f'{name}_optical_frame'
        return msg

    def destroy_node(self):
        """Release camera resources."""
        for cap in self.captures:
            if cap is not None and hasattr(cap, 'release'):
                cap.release()
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


if __name__ == '__main__':
    main()
