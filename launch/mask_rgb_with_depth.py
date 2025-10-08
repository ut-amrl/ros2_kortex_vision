#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class MaskRgbWithDepth(Node):
    def __init__(self):
        super().__init__('mask_rgb_with_depth')
        self.bridge = CvBridge()
        qos = rclpy.qos.QoSProfile(depth=10)

        # Topics
        self.rgb_sub = self.create_subscription(Image, 'color/image_rect_color', self.on_rgb, qos)
        self.dep_sub = self.create_subscription(Image, 'depth_registered/image_rect', self.on_depth, qos)
        self.pub = self.create_publisher(Image, 'rgb_masked/image', qos)

        # Params
        self.declare_parameter('depth_epsilon', 1e-6)   # for float depths: treat <= epsilon as invalid
        self.declare_parameter('min_depth_m', 0.10)     # discard < 10 cm
        self.declare_parameter('max_depth_m', 3.00)     # discard > 4 m (tune for your scene)

        self._rgb = None
        self._depth = None

    def on_rgb(self, msg):
        self._rgb = msg
        self.try_publish()

    def on_depth(self, msg):
        self._depth = msg
        self.try_publish()

    def try_publish(self):
        if self._rgb is None or self._depth is None:
            return

        rgb = self.bridge.imgmsg_to_cv2(self._rgb, desired_encoding='bgr8')
        dm = self._depth

        min_m = float(self.get_parameter('min_depth_m').value)
        max_m = float(self.get_parameter('max_depth_m').value)
        eps   = float(self.get_parameter('depth_epsilon').value)

        if dm.encoding in ('32FC1', 'TYPE_32FC1'):
            depth_m = self.bridge.imgmsg_to_cv2(dm, desired_encoding='32FC1')  # meters
            valid = np.isfinite(depth_m) & (depth_m > max(eps, min_m)) & (depth_m <= max_m)
        else:
            # assume 16UC1 in millimeters
            depth_mm = self.bridge.imgmsg_to_cv2(dm, desired_encoding='passthrough')
            valid = (depth_mm > int(min_m * 1000.0)) & (depth_mm <= int(max_m * 1000.0))

        # Safety: size mismatch guard (shouldn't happen if RegisterNode is used)
        if rgb.shape[:2] != valid.shape[:2]:
            import cv2
            valid = cv2.resize(valid.astype(np.uint8), (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_NEAREST).astype(bool)

        out = rgb.copy()
        out[~valid] = 0

        out_msg = self.bridge.cv2_to_imgmsg(out, encoding='bgr8')
        out_msg.header = self._rgb.header
        self.pub.publish(out_msg)

def main():
    rclpy.init()
    rclpy.spin(MaskRgbWithDepth())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
