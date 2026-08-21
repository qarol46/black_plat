FROM ros:humble-ros-core

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV TZ=Europe/Moscow

ENV ROS_DISTRO=humble

# Install base dependencies
RUN apt update && apt install -y \
    build-essential \
    cmake \
    python3 \
    python3-pip \
    python3-colcon-common-extensions \
    libglew-dev \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-nav2-rviz-plugins \
    ros-$ROS_DISTRO-rviz-imu-plugin \
    && rm -rf /var/lib/apt/lists/*

COPY ./docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
