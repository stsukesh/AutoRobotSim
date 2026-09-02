#!/usr/bin/env python3
"""
Autonomous Mission Commander Node
Orchestrates multi-waypoint patrol missions using Nav2 API,
integrating visual perception verification at each checkpoint.
"""

import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Bool
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def create_pose_stamped(navigator: BasicNavigator, x: float, y: float, yaw_rad: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0

    # Convert Euler yaw to Quaternion (qx, qy, qz, qw)
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
    pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
    return pose


class TargetListener(Node):
    def __init__(self):
        super().__init__('mission_target_listener')
        self.target_detected = False
        self.target_metrics = (0.0, 0.0)

        self.create_subscription(
            Bool,
            '/perception/target_detected',
            self.det_callback,
            10
        )
        self.create_subscription(
            Vector3,
            '/perception/target_bearing_distance',
            self.metrics_callback,
            10
        )

    def det_callback(self, msg: Bool):
        self.target_detected = msg.data

    def metrics_callback(self, msg: Vector3):
        self.target_metrics = (msg.x, msg.y)


def main():
    rclpy.init()

    navigator = BasicNavigator()
    listener = TargetListener()

    print("\n" + "="*60)
    print("      AUTONOMOUS ROBOTICS SIMULATION - MISSION COMMANDER")
    print("="*60)

    # 1. Set Initial Pose (Home position)
    initial_pose = create_pose_stamped(navigator, 0.0, -1.5, 1.57)
    navigator.setInitialPose(initial_pose)

    # 2. Wait for Nav2 active
    print("[MISSION] Waiting for Nav2 stack to become active...")
    navigator.waitUntilNav2Active()
    print("[MISSION] Nav2 stack is active and ready!\n")

    # 3. Define Waypoint Inspection Plan
    waypoints = [
        {
            'name': 'Waypoint 1 - Red Beacon Observation Post',
            'x': 1.0, 'y': 0.0, 'yaw': 0.0,
            'expected_target': 'Red Beacon'
        },
        {
            'name': 'Waypoint 2 - Blue Beacon Inspection Zone',
            'x': -2.2, 'y': 2.5, 'yaw': 2.35,
            'expected_target': 'Blue Beacon'
        },
        {
            'name': 'Waypoint 3 - Green Beacon Observation Post',
            'x': 2.2, 'y': -2.5, 'yaw': -0.78,
            'expected_target': 'Green Beacon'
        },
        {
            'name': 'Waypoint 4 - Home Charging Base',
            'x': 0.0, 'y': -1.5, 'yaw': 1.57,
            'expected_target': 'Home Base'
        }
    ]

    # 4. Execute Autonomous Inspection Route
    for i, wp in enumerate(waypoints, start=1):
        print(f"\n>>> [MISSION STEP {i}/{len(waypoints)}] Navigating to: {wp['name']}")
        goal_pose = create_pose_stamped(navigator, wp['x'], wp['y'], wp['yaw'])

        navigator.goToPose(goal_pose)

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                dist_remaining = feedback.distance_remaining
                print(f"    [Nav2 Progress] Distance to goal: {dist_remaining:.2f} m", end='\r')
            time.sleep(0.5)

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"\n[MISSION] Successfully reached {wp['name']}!")

            # Perform Visual Inspection Check
            print(f"[MISSION] Inspecting area for {wp['expected_target']}...")
            inspection_start = time.time()
            target_found = False

            while time.time() - inspection_start < 3.0:
                rclpy.spin_once(listener, timeout_sec=0.1)
                if listener.target_detected:
                    target_found = True
                    dist, bearing = listener.target_metrics
                    print(
                        f"    --> [PERCEPTION VERIFIED] Target detected at distance={dist:.2f}m, "
                        f"bearing={math.degrees(bearing):+.1f} deg"
                    )
                    break
                time.sleep(0.2)

            if not target_found:
                print("    --> [PERCEPTION] No target detected at this waypoint orientation.")

        elif result == TaskResult.CANCELED:
            print(f"\n[MISSION WARNING] Navigation goal canceled for {wp['name']}!")
        elif result == TaskResult.FAILED:
            print(f"\n[MISSION ERROR] Failed to navigate to {wp['name']}!")

        time.sleep(1.0)

    print("\n" + "="*60)
    print("      MISSION COMPLETED - ALL WAYPOINTS PROCESSED")
    print("="*60 + "\n")

    listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
