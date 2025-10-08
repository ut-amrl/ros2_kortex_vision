#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2

from message_filters import Subscriber, ApproximateTimeSynchronizer

class MaskRgbAlignedToDepth(Node):
    """
    Output: RGB masked by valid depth, at the DEPTH resolution & frame.

    Subscribes:
      - color/image_rect_color (or color/image_raw)  [Image, BGR8 assumed]
      - depth/image_raw                              [Image, 16UC1 mm OR 32FC1 m]
      - depth/camera_info                            [CameraInfo]

    Publishes:
      - rgb_masked/image           [Image, BGR8]  (size = depth WxH, header = depth header)
      - rgb_masked/camera_info     [CameraInfo]   (copied from depth CameraInfo, ts & frame match depth)
    """

    def __init__(self):
        super().__init__('mask_rgb_aligned_to_depth')
        self.bridge = CvBridge()

        # --- params ---
        self.declare_parameter('rgb_topic', 'color/image_rect_color')   # or color/image_raw
        self.declare_parameter('depth_topic', 'depth/image_raw')        # RAW depth (not registered)
        self.declare_parameter('depth_info_topic', 'depth/camera_info')
        self.declare_parameter('out_image_topic', 'rgb_masked/image')
        self.declare_parameter('out_info_topic', 'rgb_masked/camera_info')

        self.declare_parameter('min_depth_m', 0.10)     # ignore < 10cm
        self.declare_parameter('max_depth_m', 4.00)     # ignore > 4m
        self.declare_parameter('depth_epsilon', 1e-6)   # for 32FC1

        self.declare_parameter('sync_queue', 10)
        self.declare_parameter('sync_slop', 0.05)       # 50ms

        rgb_topic   = self.get_parameter('rgb_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        info_topic  = self.get_parameter('depth_info_topic').get_parameter_value().string_value
        out_img     = self.get_parameter('out_image_topic').get_parameter_value().string_value
        out_info    = self.get_parameter('out_info_topic').get_parameter_value().string_value

        # QoS for sensor streams
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Depth CameraInfo cache
        self._depth_info_latest = None
        self._depth_info_sub = self.create_subscription(CameraInfo, info_topic, self._on_depth_info, 10)

        # Publisher
        self.pub_img  = self.create_publisher(Image, out_img, 10)
        self.pub_info = self.create_publisher(CameraInfo, out_info, 10)

        # Synchronize RGB + depth images
        self.rgb_sub  = Subscriber(self, Image, rgb_topic, qos_profile=sensor_qos)
        self.dep_sub  = Subscriber(self, Image, depth_topic, qos_profile=sensor_qos)
        ats = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.dep_sub],
            queue_size=int(self.get_parameter('sync_queue').value),
            slop=float(self.get_parameter('sync_slop').value)
        )
        ats.registerCallback(self._on_pair)

        self.get_logger().info(
            f"Masking RGB→DEPTH: rgb='{rgb_topic}', depth='{depth_topic}', info='{info_topic}', "
            f"out='{out_img}', out_info='{out_info}'"
        )

    def _on_depth_info(self, msg: CameraInfo):
        self._depth_info_latest = msg

    def _on_pair(self, rgb_msg: Image, depth_msg: Image):
        # Convert RGB
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')

        # Build validity mask from depth
        min_m = float(self.get_parameter('min_depth_m').value)
        max_m = float(self.get_parameter('max_depth_m').value)
        eps   = float(self.get_parameter('depth_epsilon').value)

        if depth_msg.encoding in ('32FC1', 'TYPE_32FC1'):
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')  # meters
            valid = np.isfinite(depth) & (depth > max(eps, min_m)) & (depth <= max_m)
        else:
            # assume 16UC1 (mm) or passthrough that yields uint16
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            # If it came through as float but not declared, guard:
            if depth.dtype == np.float32 or depth.dtype == np.float64:
                valid = np.isfinite(depth) & (depth > max(eps, min_m)) & (depth <= max_m)
            else:
                valid = (depth > int(min_m * 1000.0)) & (depth <= int(max_m * 1000.0))

        # Resize RGB down to DEPTH resolution (width,height from depth image)
        h_d, w_d = depth.shape[:2]
        if rgb.shape[0] != h_d or rgb.shape[1] != w_d:
            rgb = cv2.resize(rgb, (w_d, h_d), interpolation=cv2.INTER_LINEAR)

        # (Optional) clean up mask: close tiny holes, remove specks
        k = np.ones((3,3), np.uint8)
        valid_u8 = (valid.astype(np.uint8) * 255)
        valid_u8 = cv2.morphologyEx(valid_u8, cv2.MORPH_CLOSE, k, iterations=1)
        valid_u8 = cv2.morphologyEx(valid_u8, cv2.MORPH_OPEN,  k, iterations=1)
        valid = valid_u8.astype(bool)

        # Apply mask
        out = rgb.copy()
        out[~valid] = 0

        # Publish masked image with DEPTH header (frame, timestamp)
        out_msg = self.bridge.cv2_to_imgmsg(out, encoding='bgr8')
        out_msg.header = depth_msg.header
        self.pub_img.publish(out_msg)

        # Publish matching CameraInfo (use latest depth CameraInfo if available)
        if self._depth_info_latest is not None:
            info = CameraInfo()
            info = self._depth_info_latest  # shallow copy is fine here
            info.header = depth_msg.header  # make timestamp/frame match the image
            self.pub_info.publish(info)

def main():
    rclpy.init()
    rclpy.spin(MaskRgbAlignedToDepth())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
