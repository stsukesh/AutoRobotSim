#!/usr/bin/env python3
"""
ROS 2 Visual Detector Node
Subscribes to camera image topic, processes frames through OpenCV perception pipeline,
and publishes target detection states, 3D target coordinates, and annotated debug images.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Bool
import cv2
import numpy as np

try:
    from cv_bridge import CvBridge
    HAS_CV_BRIDGE = True
except ImportError:
    HAS_CV_BRIDGE = False

from auto_robot_perception.cv_pipeline import VisualPerceptionPipeline, DetectionResult


class VisualDetectorNode(Node):
    def __init__(self):
        super().__init__('visual_detector_node')

        # Parameters
        self.declare_parameter('target_color', 'red')
        self.declare_parameter('min_contour_area', 400.0)
        self.declare_parameter('target_real_height_m', 0.6)
        self.declare_parameter('target_real_width_m', 0.3)
        self.declare_parameter('camera_frame', 'camera_link_optical')

        target_color = self.get_parameter('target_color').value
        min_area = self.get_parameter('min_contour_area').value
        target_h = self.get_parameter('target_real_height_m').value
        target_w = self.get_parameter('target_real_width_m').value
        self.camera_frame = self.get_parameter('camera_frame').value

        # Initialize CV pipeline
        self.pipeline = VisualPerceptionPipeline(
            target_color=target_color,
            min_contour_area=min_area,
            target_real_height_m=target_h,
            target_real_width_m=target_w
        )

        if HAS_CV_BRIDGE:
            self.bridge = CvBridge()
        else:
            self.bridge = None
            self.get_logger().warn('cv_bridge not found; using fallback raw buffer converter.')

        # QoS Profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Publishers
        self.pub_detected = self.create_publisher(Bool, '/perception/target_detected', 10)
        self.pub_pose = self.create_publisher(PoseStamped, '/perception/target_pose', 10)
        self.pub_metrics = self.create_publisher(Vector3, '/perception/target_bearing_distance', 10)
        self.pub_annotated_img = self.create_publisher(Image, '/perception/image_annotated', 10)

        # Subscribers
        self.sub_cam_info = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            sensor_qos
        )

        self.sub_image = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            sensor_qos
        )

        self.get_logger().info(
            f'Visual Detector Node started. Tracking [{target_color.upper()}] targets.'
        )

    def camera_info_callback(self, msg: CameraInfo):
        """Update intrinsics matrix from camera info."""
        if len(msg.k) == 9:
            fx = msg.k[0]
            cx = msg.k[2]
            fy = msg.k[4]
            cy = msg.k[5]
            if fx > 0.0:
                self.pipeline.update_intrinsics(fx, fy, cx, cy)

    def imgmsg_to_cv2(self, img_msg: Image) -> np.ndarray:
        """Convert ROS Image message to OpenCV BGR numpy array."""
        if self.bridge is not None:
            return self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        else:
            # Fallback manual converter
            if img_msg.encoding in ['rgb8', 'bgr8']:
                img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape((img_msg.height, img_msg.width, 3))
                if img_msg.encoding == 'rgb8':
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                return img
            else:
                raise ValueError(f'Unsupported encoding: {img_msg.encoding}')

    def cv2_to_imgmsg(self, cv_image: np.ndarray, frame_id: str) -> Image:
        """Convert OpenCV BGR image to ROS Image message."""
        if self.bridge is not None:
            msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            msg.header.frame_id = frame_id
            msg.header.stamp = self.get_clock().now().to_msg()
            return msg
        else:
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = frame_id
            msg.height = cv_image.shape[0]
            msg.width = cv_image.shape[1]
            msg.encoding = 'bgr8'
            msg.is_bigendian = 0
            msg.step = cv_image.shape[1] * 3
            msg.data = cv_image.tobytes()
            return msg

    def image_callback(self, msg: Image):
        try:
            cv_img = self.imgmsg_to_cv2(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to decode camera image: {e}')
            return

        result, annotated_img = self.pipeline.process_frame(cv_img)

        # 1. Publish detection boolean
        det_msg = Bool()
        det_msg.data = result.detected
        self.pub_detected.publish(det_msg)

        # 2. Publish metrics (Distance, Bearing in rad, Contour Area)
        metrics_msg = Vector3()
        if result.detected:
            metrics_msg.x = result.distance_m
            metrics_msg.y = result.bearing_rad
            metrics_msg.z = result.contour_area
        else:
            metrics_msg.x = 0.0
            metrics_msg.y = 0.0
            metrics_msg.z = 0.0
        self.pub_metrics.publish(metrics_msg)

        # 3. Publish 3D Pose in camera frame
        if result.detected:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = self.camera_frame
            pose_msg.pose.position.x = result.position_camera_frame[0]
            pose_msg.pose.position.y = result.position_camera_frame[1]
            pose_msg.pose.position.z = result.position_camera_frame[2]
            pose_msg.pose.orientation.w = 1.0  # Default orientation
            self.pub_pose.publish(pose_msg)

        # 4. Publish annotated visualization stream
        try:
            annotated_msg = self.cv2_to_imgmsg(annotated_img, self.camera_frame)
            self.pub_annotated_img.publish(annotated_msg)
        except Exception as e:
            self.get_logger().debug(f'Failed to publish annotated image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = VisualDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
