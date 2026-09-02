FROM osrf/ros:humble-desktop-full

# Avoid interactive prompts during apt install
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV SHELL=/bin/bash

# Install essential build tools, Gazebo classic, Nav2, SLAM Toolbox, robot_localization, OpenCV, and utilities
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
    # Gazebo Classic & ROS 2 integration
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-plugins \
    ros-humble-gazebo-msgs \
    \
    # Robot State & Control
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-teleop-twist-keyboard \
    \
    # Sensor Fusion & Localization
    ros-humble-robot-localization \
    \
    # SLAM & Navigation 2
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-simple-commander \
    \
    # Vision & Perception
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    python3-opencv \
    python3-numpy \
    \
    # Visualization & Tools
    ros-humble-rviz2 \
    ros-humble-rqt* \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    x11-apps \
    nano \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install python dependencies for perception and testing
RUN pip3 install --no-cache-dir \
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
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /workspace/ros2_ws/install/setup.bash ]; then source /workspace/ros2_ws/install/setup.bash; fi" >> /home/$USERNAME/.bashrc

# Set workspace directory
WORKDIR /workspace

# Set entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER $USERNAME
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
