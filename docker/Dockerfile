FROM osrf/ros:jazzy-desktop-full

# Avoid interactive prompts during apt install
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV SHELL=/bin/bash
ENV GZ_VERSION=harmonic

# Install essential build tools, Gazebo Harmonic, Nav2, SLAM Toolbox, robot_localization, OpenCV, and utilities
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    \
    # Gazebo Harmonic & ROS 2 integration (modern Gazebo via ros_gz) \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-image \
    \
    # Robot State & Control \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    ros-jazzy-teleop-twist-keyboard \
    \
    # Sensor Fusion & Localization \
    ros-jazzy-robot-localization \
    \
    # SLAM & Navigation 2 \
    ros-jazzy-slam-toolbox \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-simple-commander \
    ros-jazzy-nav2-mppi-controller \
    \
    # Vision & Perception \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-image-transport-plugins \
    python3-opencv \
    python3-numpy \
    \
    # Visualization & Tools \
    ros-jazzy-rviz2 \
    ros-jazzy-rqt* \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    x11-apps \
    nano \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install python dependencies for perception and testing
RUN pip3 install --no-cache-dir --break-system-packages \
    pytest \
    scipy \
    matplotlib \
    transforms3d

# Create user with UID 1000 to match host permissions
ARG USERNAME=robot
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME 2>/dev/null || true \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS environment in bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /workspace/ros2_ws/install/setup.bash ]; then source /workspace/ros2_ws/install/setup.bash; fi" >> /home/$USERNAME/.bashrc

# Set workspace directory
WORKDIR /workspace

# Set entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER $USERNAME
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
