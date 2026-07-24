FROM t21-rviz2

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV TZ=Europe/Moscow
ENV ROS_DISTRO=humble

# ── Lidar & clahe_ros deps ────────────────────────────────────────────────────
RUN apt-get update --fix-missing && \
    apt-get install --fix-missing -y \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-velodyne-msgs \
    ros-$ROS_DISTRO-velodyne-pointcloud \
    ros-$ROS_DISTRO-point-cloud-interfaces \
    ros-$ROS_DISTRO-point-cloud-msg-wrapper \
    ros-$ROS_DISTRO-pcl-conversions \
    ros-$ROS_DISTRO-pcl-msgs \
    ros-$ROS_DISTRO-pcl-ros \
    ros-$ROS_DISTRO-perception-pcl \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-image-transport-plugins \
    ros-$ROS_DISTRO-message-filters \
    libopencv-dev \
    python3-opencv && \
    pip install open3d scipy numba "numpy<2.0" && \
    rm -rf /var/lib/apt/lists/*

# ── Sources & build ────────────────────────────────────────────────────────────
COPY ./ros2 /ros2_ws/src
COPY ./docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

RUN /bin/bash -c '\
    . /opt/ros/$ROS_DISTRO/setup.bash && \
    cd /ros2_ws && \
    colcon build --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo'

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]