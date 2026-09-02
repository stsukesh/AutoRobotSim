# Troubleshooting & Diagnostic Guide

This guide documents common issues encountered during ROS 2 mobile robotics simulation, their root causes, and verified solutions.

---

## 1. Coordinate Transform (TF) Issues

### Problem 1.1: `Transform [sender=unknown] between [/odom] and [/base_footprint] was not found`
- **Cause**: Both the Gazebo differential drive plugin and the `robot_localization` EKF filter might be attempting to publish `odom -> base_footprint`, creating TF conflicts, or the EKF filter node has not started.
- **Fix**:
  1. Ensure `publish_odom_tf` in `gazebo_control.xacro` is set to `false`.
  2. Verify that `robot_localization` is running with `publish_tf: true`.
  3. Inspect the active TF tree:
     ```bash
     ros2 run tf2_tools view_frames
     evince frames.pdf
     ```

### Problem 1.2: Camera Optical Link Inversion in RViz
- **Cause**: Standard ROS camera coordinate conventions require $+Z$ pointing forward into the scene, $+X$ right, and $+Y$ down. If the standard robot link is used directly for image transport, the 3D point cloud or vision markers will appear rotated 90 degrees.
- **Fix**: Maintain a dedicated `camera_link_optical` joint with rotation `rpy="-1.5708 0 -1.5708"` relative to `camera_link`.

---

## 2. Quality of Service (QoS) Mismatches

### Problem 2.1: Node Not Receiving `/scan` or `/camera/image_raw` Messages
- **Cause**: Gazebo publishes high-throughput sensor topics using `BEST_EFFORT` reliability and `VOLATILE` durability. If a subscriber is instantiated with default `RELIABLE` QoS, ROS 2 will silently drop incoming messages due to QoS incompatibility.
- **Fix**: Explicitly set the subscriber QoS profile:
  ```python
  from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

  sensor_qos = QoSProfile(
      reliability=ReliabilityPolicy.BEST_EFFORT,
      history=HistoryPolicy.KEEP_LAST,
      depth=10
  )
  self.create_subscription(LaserScan, '/scan', self.scan_cb, sensor_qos)
  ```

---

## 3. Simulation & Clock Synchronization

### Problem 3.1: Nav2 or Nodes Hanging Waiting for Time
- **Cause**: When running in Gazebo, ROS nodes must synchronize with the simulated `/clock` topic published by Gazebo (`use_sim_time:=true`). If one node uses wall clock time while others use sim time, TF lookups will fail with `ExtrapolationException (lookup would require extrapolation into the future)`.
- **Fix**: Always pass `use_sim_time: true` to all launch files and parameter YAML files.

---

## 4. Nav2 Path Planning & Obstacle Costmap Issues

### Problem 4.1: Robot Stuck Rotating in Place (Recovery Behavior Loop)
- **Cause**: The costmap inflation radius or obstacle padding is too large, causing narrow doorways to be marked as lethal obstacles in the costmap.
- **Fix**:
  1. Adjust `inflation_radius` from `0.65m` down to `0.45m` in `nav2_params.yaml`.
  2. Adjust `robot_radius` to accurately match the robot bounding cylinder ($0.22\text{m}$).
  3. Clear costmaps via ROS 2 service call:
     ```bash
     ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}"
     ```

---

## 5. Docker & GUI X11 Forwarding Issues

### Problem 5.1: `Could not connect to display :0` or `cannot open display`
- **Cause**: The host X server is blocking container access to the X11 socket.
- **Fix**:
  1. On the host terminal, run:
     ```bash
     xhost +local:root
     xhost +local:$USER
     ```
  2. Verify that `/tmp/.X11-unix` is mounted into the container in `docker-compose.yml`.
