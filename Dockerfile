FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# ----- System utilities -----
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-setuptools \
    wget curl git \
    && rm -rf /var/lib/apt/lists/*

# ----- Python packages -----
RUN pip3 install --no-cache-dir \
    transforms3d \
    "numpy<2" \
    opencv-python-headless

# ----- MoveIt 2 -----
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-moveit \
    ros-humble-moveit-configs-utils \
    ros-humble-moveit-ros-move-group \
    ros-humble-moveit-ros-planning-interface \
    && rm -rf /var/lib/apt/lists/*

# ----- ros2_control stack -----
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-trajectory-controller \
    ros-humble-joint-state-broadcaster \
    ros-humble-controller-manager \
    && rm -rf /var/lib/apt/lists/*

# ----- Gazebo Ignition Fortress + ROS bridges -----
RUN apt-get update && apt-get install -y --no-install-recommends \
    ignition-fortress \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-image \
    ros-humble-ign-ros2-control \
    && rm -rf /var/lib/apt/lists/*

# ----- Vision / TF -----
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-tf2-ros \
    ros-humble-tf-transformations \
    && rm -rf /var/lib/apt/lists/*

# ----- Robot description (Franka Panda) -----
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-franka-description \
    ros-humble-moveit-resources-panda-description \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# ----- Copy and build workspace -----
WORKDIR /ros2_ws

COPY src/ src/

RUN rosdep update && \
    . /opt/ros/humble/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y || true

# Build without --symlink-install so install/ is fully self-contained
RUN . /opt/ros/humble/setup.sh && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    && rm -rf log/ build/

# ----- Entrypoint -----
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
