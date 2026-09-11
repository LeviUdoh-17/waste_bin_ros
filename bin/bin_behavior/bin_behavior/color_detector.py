import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

BLUE_LOWER  = np.array([100, 60, 60])
BLUE_UPPER  = np.array([130, 255, 255])
GREEN_LOWER = np.array([40, 60, 60])
GREEN_UPPER = np.array([80, 255, 255])
MIN_PIXELS  = 500       # minimum contiguous area (pixels) to count as a detection
COOLDOWN_S  = 5.0       # seconds before re-triggering is allowed

_BLUE_BGR  = (255,   0,   0)
_GREEN_BGR = (  0, 255,   0)
_FONT      = cv2.FONT_HERSHEY_SIMPLEX


class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        self._bridge = CvBridge()
        self._pub     = self.create_publisher(Bool,  '/bin_trigger',              10)
        self._img_pub = self.create_publisher(Image, '/color_detector/debug_image', 10)
        self.create_subscription(Image, '/camera/rgb/image_raw', self._cb, 10)
        self._last_trigger = self.get_clock().now().nanoseconds * 1e-9 - COOLDOWN_S
        self.get_logger().info('ColorDetector ready')

    def _cb(self, msg):
        now   = self.get_clock().now().nanoseconds * 1e-9
        frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        blue  = cv2.inRange(hsv, BLUE_LOWER,  BLUE_UPPER)
        green = cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)

        # Find qualifying contours in each mask
        qualified = False
        for mask, color in ((blue, _BLUE_BGR), (green, _GREEN_BGR)):
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                if cv2.contourArea(c) < MIN_PIXELS:
                    continue
                qualified = True
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                label_y = max(y - 10, 15)
                cv2.putText(frame, 'Detected Target', (x, label_y),
                            _FONT, 0.6, color, 2, cv2.LINE_AA)

        # Publish annotated frame every cycle (regardless of cooldown)
        self._img_pub.publish(self._bridge.cv2_to_imgmsg(frame, 'bgr8'))

        # Fire trigger only when a detection exists and cooldown has passed
        if qualified and now - self._last_trigger >= COOLDOWN_S:
            self._pub.publish(Bool(data=True))
            self._last_trigger = now
            self.get_logger().info('Color trigger fired')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ColorDetector())
    rclpy.shutdown()
