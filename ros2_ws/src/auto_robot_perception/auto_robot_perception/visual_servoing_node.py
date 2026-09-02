#!/usr/bin/env python3
"""
Visual Servoing & Target Following Node
Implements closed-loop Proportional-Derivative (PD) control to orient the robot towards
the visual target and maintain a specified stand-off distance.
Integrates LiDAR collision safety override for robust simulation navigation.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class VisualServoingNode(Node):
    def __init__(self):
        super().__init__('visual_servoing_node')

        # Control parameters
        self.declare_parameter('target_distance_m', 0.8)
        self.declare_parameter('kp_linear', 0.5)
        self.declare_parameter('kd_linear', 0.1)
        self.declare_parameter('kp_angular', 1.2)
        self.declare_parameter('kd_angular', 0.15)
        self.declare_parameter('max_linear_vel', 0.4)
        self.declare_parameter('max_angular_vel', 0.8)
        self.declare_parameter('safety_stop_distance_m', 0.35)
        self.declare_parameter('search_spin_speed', 0.25)
        self.declare_parameter('enable_search_mode', True)

        self.target_dist = self.get_parameter('target_distance_m').value
        self.kp_lin = self.get_parameter('kp_linear').value
        self.kd_lin = self.get_parameter('kd_linear').value
        self.kp_ang = self.get_parameter('kp_angular').value
        self.kd_ang = self.get_parameter('kd_angular').value
        self.max_lin = self.get_parameter('max_linear_vel').value
        self.max_ang = self.get_parameter('max_angular_vel').value
        self.safety_dist = self.get_parameter('safety_stop_distance_m').value
        self.search_spin = self.get_parameter('search_spin_speed').value
        self.search_mode = self.get_parameter('enable_search_mode').value

        # State variables
        self.target_detected = False
        self.curr_distance = 0.0
        self.curr_bearing = 0.0
        self.prev_dist_error = 0.0
        self.prev_bearing_error = 0.0
        self.min_obstacle_front = 10.0
        self.last_detection_time = self.get_clock().now()

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.sub_detected = self.create_subscription(
            Bool,
            '/perception/target_detected',
            self.detected_callback,
            10
        )

        self.sub_metrics = self.create_subscription(
            Vector3,
            '/perception/target_bearing_distance',
            self.metrics_callback,
            10
        )

        self.sub_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            sensor_qos
        )

        # 20 Hz Control Loop Timer
        self.control_timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f'Visual Servoing Controller Active. Target distance: {self.target_dist}m'
        )

    def detected_callback(self, msg: Bool):
        self.target_detected = msg.data
        if msg.data:
            self.last_detection_time = self.get_clock().now()

    def metrics_callback(self, msg: Vector3):
        self.curr_distance = msg.x
        self.curr_bearing = msg.y

    def scan_callback(self, msg: LaserScan):
        """Monitor frontal sector of LiDAR for obstacle safety override."""
        if not msg.ranges:
            return

        num_readings = len(msg.ranges)
        # Frontal cone: -30 deg to +30 deg
        front_indices = list(range(0, int(num_readings * 30 / 360))) + \
                        list(range(int(num_readings * 330 / 360), num_readings))

        valid_ranges = [
            msg.ranges[i] for i in front_indices
            if msg.range_min < msg.ranges[i] < msg.range_max and not math.isnan(msg.ranges[i])
        ]

        if valid_ranges:
            self.min_obstacle_front = min(valid_ranges)
        else:
            self.min_obstacle_front = 10.0

    def control_loop(self):
        cmd = Twist()
        dt = 0.05

        time_since_last_detection = (
            self.get_clock().now() - self.last_detection_time
        ).nanoseconds / 1e9

        # Safety Override Check
        if self.min_obstacle_front < self.safety_dist:
            self.get_logger().warn(
                f'LIDAR SAFETY OVERRIDE: Obstacle at {self.min_obstacle_front:.2f}m! Stopping robot.',
                throttle_duration_sec=1.0
            )
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            return

        if self.target_detected and time_since_last_detection < 0.5:
            # Target is actively tracked
            dist_error = self.curr_distance - self.target_dist
            bearing_error = self.curr_bearing

            # PD Linear Control
            d_dist = (dist_error - self.prev_dist_error) / dt
            v_lin = (self.kp_lin * dist_error) + (self.kd_lin * d_dist)
            self.prev_dist_error = dist_error

            # PD Angular Control (steer towards target center)
            d_bearing = (bearing_error - self.prev_bearing_error) / dt
            v_ang = (-self.kp_ang * bearing_error) - (self.kd_ang * d_bearing)
            self.prev_bearing_error = bearing_error

            # Clamp velocities
            cmd.linear.x = float(max(-self.max_lin, min(self.max_lin, v_lin)))
            cmd.angular.z = float(max(-self.max_ang, min(self.max_ang, v_ang)))

            # If robot is close enough to target distance, stop advancing
            if abs(dist_error) < 0.05:
                cmd.linear.x = 0.0

            # If bearing error is large, prioritize rotation before translation
            if abs(bearing_error) > 0.35:
                cmd.linear.x *= 0.2

        else:
            # Target is lost
            if self.search_mode:
                # Slowly spin to search for target
                cmd.linear.x = 0.0
                cmd.angular.z = float(self.search_spin)
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
