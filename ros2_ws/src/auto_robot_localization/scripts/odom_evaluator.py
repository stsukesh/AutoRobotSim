#!/usr/bin/env python3
"""
Odometry Evaluator Node
Compares Raw Wheel Odometry vs. EKF Filtered Odometry against Ground Truth (or evaluates relative drift).
Publishes metrics and diagnostic logs to quantify sensor fusion performance.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String


class OdomEvaluator(Node):
    def __init__(self):
        super().__init__('odom_evaluator')

        self.declare_parameter('robot_name', 'auto_robot')
        self.declare_parameter('eval_interval_sec', 2.0)

        self.robot_name = self.get_parameter('robot_name').value
        eval_interval = self.get_parameter('eval_interval_sec').value

        # Data storage
        self.raw_odom_pose = None
        self.filtered_odom_pose = None
        self.ground_truth_pose = None

        # Tracking metrics
        self.raw_errors = []
        self.ekf_errors = []
        self.step_count = 0

        # QoS Profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.sub_raw_odom = self.create_subscription(
            Odometry,
            '/odom_raw',
            self.raw_odom_callback,
            10
        )

        self.sub_filtered_odom = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.filtered_odom_callback,
            10
        )

        self.sub_ground_truth = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.ground_truth_callback,
            sensor_qos
        )

        # Metrics Publisher
        self.metrics_pub = self.create_publisher(String, '/localization/drift_metrics', 10)

        # Evaluation Timer
        self.timer = self.create_timer(eval_interval, self.evaluate_and_log)

        self.get_logger().info('Odometry Evaluator node initialized.')

    def raw_odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        self.raw_odom_pose = (p.x, p.y, p.z)

    def filtered_odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        self.filtered_odom_pose = (p.x, p.y, p.z)

    def ground_truth_callback(self, msg: ModelStates):
        try:
            if self.robot_name in msg.name:
                idx = msg.name.index(self.robot_name)
                p = msg.pose[idx].position
                self.ground_truth_pose = (p.x, p.y, p.z)
        except Exception as e:
            self.get_logger().debug(f'Error reading ground truth: {e}')

    def calculate_distance(self, p1, p2):
        if p1 is None or p2 is None:
            return 0.0
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def evaluate_and_log(self):
        if self.raw_odom_pose is None or self.filtered_odom_pose is None:
            return

        ref_pose = self.ground_truth_pose if self.ground_truth_pose is not None else (0.0, 0.0, 0.0)

        if self.ground_truth_pose is not None:
            raw_err = self.calculate_distance(self.raw_odom_pose, self.ground_truth_pose)
            ekf_err = self.calculate_distance(self.filtered_odom_pose, self.ground_truth_pose)

            self.raw_errors.append(raw_err)
            self.ekf_errors.append(ekf_err)

            raw_rmse = np.sqrt(np.mean(np.square(self.raw_errors)))
            ekf_rmse = np.sqrt(np.mean(np.square(self.ekf_errors)))

            improvement_pct = 0.0
            if raw_rmse > 0.0001:
                improvement_pct = ((raw_rmse - ekf_rmse) / raw_rmse) * 100.0

            log_msg = (
                f"\n--- [SENSOR FUSION & LOCALIZATION METRICS] ---\n"
                f"Ground Truth: X={ref_pose[0]:.3f}, Y={ref_pose[1]:.3f}\n"
                f"Raw Odometry Error: Current={raw_err:.4f}m | Cumulative RMSE={raw_rmse:.4f}m\n"
                f"EKF Filtered Error: Current={ekf_err:.4f}m | Cumulative RMSE={ekf_rmse:.4f}m\n"
                f"Drift Reduction: {improvement_pct:.2f}%\n"
                f"----------------------------------------------"
            )
            self.get_logger().info(log_msg)

            metrics_msg = String()
            metrics_msg.data = f'{{"raw_rmse": {raw_rmse:.4f}, "ekf_rmse": {ekf_rmse:.4f}, "improvement_pct": {improvement_pct:.2f}}}'
            self.metrics_pub.publish(metrics_msg)
        else:
            diff = self.calculate_distance(self.raw_odom_pose, self.filtered_odom_pose)
            self.get_logger().info(
                f"[LOCALIZATION] Raw vs EKF pose difference: {diff:.4f}m "
                f"(Raw: ({self.raw_odom_pose[0]:.2f}, {self.raw_odom_pose[1]:.2f}) | "
                f"EKF: ({self.filtered_odom_pose[0]:.2f}, {self.filtered_odom_pose[1]:.2f}))"
            )


def main(args=None):
    rclpy.init(args=args)
    node = OdomEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
