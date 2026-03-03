FROM t21-rviz2

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV TZ=Europe/Moscow

ENV ROS_DISTRO=humble

# Install base dependencies
RUN apt-get update && apt-get install -y \
    gazebo \
    gazebo-common \
    gazebo-plugin-base \
    libgazebo-dev \
    ros-$ROS_DISTRO-control-msgs \
    ros-$ROS_DISTRO-control-toolbox \
    ros-$ROS_DISTRO-controller-interface \
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-controller-manager-msgs \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-control-test-assets \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-hardware-interface \
    ros-$ROS_DISTRO-diff-drive-controller \
    ros-$ROS_DISTRO-effort-controllers \
    ros-$ROS_DISTRO-kinematics-interface \
    ros-$ROS_DISTRO-velocity-controllers \
    ros-$ROS_DISTRO-joint-limits \
    ros-$ROS_DISTRO-joint-state-broadcaster \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-joint-trajectory-controller \
    ros-$ROS_DISTRO-gazebo-dev \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-gazebo-msgs \
    ros-$ROS_DISTRO-gazebo-plugins \
    ros-$ROS_DISTRO-gazebo-ros \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-realsense2-description \
    ros-$ROS_DISTRO-gazebo-ros2-control && \
    rm -rf /var/lib/apt/lists/*

COPY ./ros2 /ros2_ws/src
COPY ./docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash; cd ros2_ws && colcon build --symlink-install'

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]