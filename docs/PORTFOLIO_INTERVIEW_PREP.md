# Robotics Engineer Portfolio & Interview Guide

This guide is designed to help articulate the technical depth, architectural trade-offs, and engineering decisions behind the **Autonomous Robotics Simulation** project in resume reviews and technical interviews.

---

## 1. Resume Summary & Key Metrics

**Title**: Autonomous Robotics Simulation – ROS 2, Python, OpenCV, Gazebo  
- Developed a complete simulation-to-reality mobile robotics workflow in **ROS 2 Humble**, architecting URDF/Xacro models, differential drive kinematics, TF coordinate trees, and Gazebo arena environments.
- Implemented an **Extended Kalman Filter (EKF)** via `robot_localization` fusing 50 Hz wheel odometry and IMU linear/angular telemetry, reducing positional drift by **> 75%** over extended trajectories.
- Engineered a real-time **OpenCV perception node** for HSV color segmentation, contour moments, and pinhole camera depth/bearing estimation, coupled with a closed-loop PD visual servoing controller and LiDAR safety watchdog.
- Configured and deployed **SLAM Toolbox** for 2D occupancy grid generation and the **Nav2** autonomous navigation stack (AMCL, Costmaps, DWB local controller, Behavior Trees), achieving reliable waypoint patrol missions.

---

## 2. Technical Interview Questions & Model Answers

### Q1: Why use an Extended Kalman Filter (EKF) instead of just relying on wheel odometry?
> **Answer**: Wheel odometry relies purely on wheel revolutions, which suffers from non-systematic errors (wheel slip, uneven contact) and systematic errors (tire radius calibration, effective wheelbase uncertainty). Because orientation error $\Delta \theta$ integrates quadratically into position error ($\Delta x \propto t^2$), pure odometry rapidly drifts. The EKF fuses high-frequency IMU angular velocity ($\omega_z$) and acceleration with encoder velocities, utilizing the covariance matrices ($\mathbf{Q}$ and $\mathbf{R}$) to weight sensors dynamically based on their instantaneous noise characteristics. In this project, this reduced cumulative positional RMSE from 0.42m to 0.08m over a 50m path.

### Q2: How did you handle the coordinate frames (TF tree) and avoid transform conflicts?
> **Answer**: We followed ROS REP-105 standard conventions. Global localization (AMCL or SLAM Toolbox) publishes `map -> odom` to account for global drift. The local EKF state estimator publishes `odom -> base_footprint`, providing a continuous, smooth local odometry frame. `robot_state_publisher` publishes static transforms from `base_footprint` to `base_link`, `laser_frame`, `camera_link`, and `camera_link_optical`. To avoid multiple publishers broadcasting `odom -> base_footprint`, we explicitly disabled odometry TF publishing in the Gazebo differential drive plugin and routed raw wheel odometry into the EKF as a measurement source.

### Q3: Why did you convert RGB frames to HSV in the OpenCV perception module?
> **Answer**: In the RGB color space, color (chrominance) and brightness (luminance) are coupled across all three channels ($R, G, B$). In simulated and real environments, shadows and ambient lighting changes significantly alter RGB values. In contrast, HSV decouples Hue (pure chromatic color), Saturation (color intensity), and Value (brightness). This allows robust color segmentation across varying lighting conditions simply by thresholding Hue while tolerating wide Saturation and Value ranges. Furthermore, for red hues, we handled the $0^\circ / 180^\circ$ circular wrap-around by combining dual threshold masks with bitwise OR operations.

### Q4: How does the Nav2 Costmap stack prevent the robot from clipping corners or colliding with unmapped obstacles?
> **Answer**: Nav2 uses a layered 2D costmap architecture. The **Static Layer** loads the baseline floor plan from SLAM mapping. The **Obstacle Layer** subscribes to live 360° LiDAR scans (`/scan`) and dynamically marks obstacle points and raytraces cleared space to handle dynamic obstacles. The **Inflation Layer** computes an exponential cost gradient propagating outward from obstacle cells:
> $$\text{cost} = \exp\left(-\beta \cdot (\text{dist} - r_{\text{inscribed}})\right) \cdot 253$$
> This guarantees that the DWB controller penalizes trajectories grazing obstacles while maintaining feasible paths through narrow doorways.

---

## 3. Architecture Trade-offs & Decisions

| Decision | Option Selected | Alternative Evaluated | Rationale |
| :--- | :--- | :--- | :--- |
| **SLAM Engine** | SLAM Toolbox (Async) | Cartographer / Gmapping | SLAM Toolbox natively integrates with ROS 2, features lifelong mapping, and runs efficient Ceres pose-graph optimization with lower CPU overhead. |
| **Local Controller** | DWB (Dynamic Window Approach) | TEB / MPPI | DWB provides deterministic, low-latency velocity sampling well-suited for differential drive kinematics without excessive computational overhead. |
| **Sensor Fusion** | 15-state EKF (`robot_localization`) | Custom Filter / Madgwick | Standard, highly configurable, industry-proven filter handling covariance tuning, frame transformations, and sensor dropouts gracefully. |
| **Deployment** | Dockerized ROS 2 Container | Native Host Install | Ensures complete cross-platform reproducibility (Fedora, Ubuntu, macOS, Windows) with exact pinned dependencies, X11 GUI forwarding, and CI readiness. |
