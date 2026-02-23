FROM osrf/ros:humble-desktop-full

# Gerekli sistem bağımlılıkları
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-plugins \
    ros-humble-cv-bridge \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# Python bağımlılıkları
RUN pip3 install \
    opencv-python \
    pyzbar

# Workspace oluştur
WORKDIR /ros2_ws
COPY src/ src/

# Build
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
