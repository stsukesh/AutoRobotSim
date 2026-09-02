# System Architecture & Technical Design

## 1. High-Level Architecture Overview

The **Autonomous Robotics Simulation** repository implements a modular, production-grade ROS 2 autonomous mobile robot system. The pipeline integrates physics simulation, multi-sensor perception, Kalman filtering, SLAM, global/local path planning, and autonomous mission dispatching.

```mermaid
graph TD
    subgraph Gazebo Simulation
        Physics[ODE Physics Engine]
        DiffDrivePlugin[libgazebo_ros_diff_drive.so]
        LidarPlugin[libgazebo_ros_ray_sensor.so]
        CameraPlugin[libgazebo_ros_camera.so]
        ImuPlugin[libgazebo_ros_imu_sensor.so]
    end

    subgraph Hardware Abstraction & Sensor Streams
        DiffDrivePlugin -->|/odom_raw| EKF[Robot Localization EKF]
        DiffDrivePlugin -->|/joint_states| RSP[Robot State Publisher]
        ImuPlugin -->|/imu/data| EKF
        LidarPlugin -->|/scan| SLAM[SLAM Toolbox / AMCL]
        LidarPlugin -->|/scan| Costmap[Nav2 Costmaps]
        LidarPlugin -->|/scan| ServoingSafety[Visual Servoing Watchdog]
        CameraPlugin -->|/camera/image_raw| CVNode[OpenCV Visual Detector]
        CameraPlugin -->|/camera/camera_info| CVNode
    end

    subgraph State Estimation & Coordinate Frames
        EKF -->|/odometry/filtered| Nav2Controller[Nav2 DWB Controller]
        EKF -->|TF: odom -> base_footprint| TF[TF2 Coordinate Tree]
        RSP -->|TF: base_footprint -> links| TF
        SLAM -->|TF: map -> odom| TF
    end

    subgraph Perception Pipeline
        CVNode -->|/perception/target_detected| Mission[Mission Commander]
        CVNode -->|/perception/target_bearing_distance| Servoing[Visual Servoing Node]
        CVNode -->|/perception/target_pose| TF
        CVNode -->|/perception/image_annotated| RViz[RViz2 Visualization]
    end

    subgraph Autonomous Navigation & Planning
        Mission -->|Nav2 Action Goals| Nav2Stack[Nav2 BT Navigator]
        Nav2Stack --> GlobalPlanner[NavFn Global Planner]
        Nav2Stack --> Costmap
        Nav2Stack --> Nav2Controller
        Nav2Controller -->|/cmd_vel| DiffDrivePlugin
        Servoing -->|/cmd_vel_vision| DiffDrivePlugin
    end
```

---

## 2. Coordinate Transformations (TF Tree)

Standard ROS REP-105 compliance is strictly maintained:

$$\text{map} \xrightarrow[\text{SLAM / AMCL}]{} \text{odom} \xrightarrow[\text{robot\_localization EKF}]{} \text{base\_footprint} \xrightarrow[\text{URDF Static}]{} \text{base\_link} \xrightarrow[\text{URDF Static}]{} \begin{cases} \text{chassis} \to \text{laser\_frame} \\ \text{chassis} \to \text{camera\_link} \to \text{camera\_link\_optical} \\ \text{chassis} \to \text{imu\_link} \\ \text{base\_link} \to \text{left\_wheel} \\ \text{base\_link} \to \text{right\_wheel} \end{cases}$$

### Frame Definitions:
- **`map`**: World-fixed coordinate frame with origin at the global mapping datum. Global localization algorithms (AMCL / SLAM Toolbox) publish the `map -> odom` transformation to correct for long-term odometric drift.
- **`odom`**: World-fixed coordinate frame that is continuous and locally smooth (without discrete jumps). The Extended Kalman Filter (`robot_localization`) publishes the `odom -> base_footprint` transformation by fusing wheel encoders and IMU.
- **`base_footprint`**: 2D planar projection of the robot chassis on the ground surface ($Z = 0$).
- **`base_link`**: Center of the robot's differential drive axle.
- **`camera_link_optical`**: Optical camera frame rotated by $-90^\circ$ pitch and $-90^\circ$ yaw to adhere to standard camera convention ($+Z$ forward into the scene, $+X$ right, $+Y$ down).

---

## 3. Sensor Pipeline Specifications

| Sensor Type | ROS 2 Topic | Message Type | Update Rate | Purpose |
| :--- | :--- | :--- | :--- | :--- |
| **2D LiDAR** | `/scan` | `sensor_msgs/msg/LaserScan` | 10 Hz | 360° obstacle avoidance, SLAM mapping, and AMCL particle localization |
| **IMU** | `/imu/data` | `sensor_msgs/msg/Imu` | 50 Hz | High-rate angular velocity ($\omega_z$) and linear acceleration ($a_x, a_y$) for EKF fusion |
| **Monocular Camera** | `/camera/image_raw` | `sensor_msgs/msg/Image` | 30 Hz | RGB image feed (640x480) for OpenCV color segmentation and visual servoing |
| **Camera Info** | `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 30 Hz | Intrinsic matrix ($f_x, f_y, c_x, c_y$) and distortion parameters |
| **Wheel Odometry** | `/odom_raw` | `nav_msgs/msg/Odometry` | 50 Hz | Encoder-based differential drive displacement and velocities |
| **Fused Odometry** | `/odometry/filtered` | `nav_msgs/msg/Odometry` | 30 Hz | State-estimated pose and twist from EKF filter |
| **Vision Telemetry** | `/perception/target_bearing_distance` | `geometry_msgs/msg/Vector3` | 30 Hz | Estimated distance ($m$), bearing angle ($\text{rad}$), and contour area |
