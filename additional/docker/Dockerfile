FROM t21-rviz2

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV TZ=Europe/Moscow

ENV ROS_DISTRO=humble

RUN apt-get update --fix-missing

# Install base dependencies
RUN apt-get install -y \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-velodyne-msgs \
    ros-$ROS_DISTRO-velodyne-pointcloud \
    ros-$ROS_DISTRO-point-cloud-interfaces \
    ros-$ROS_DISTRO-point-cloud-msg-wrapper \
    ros-$ROS_DISTRO-pcl-conversions \
    ros-$ROS_DISTRO-pcl-msgs \
    ros-$ROS_DISTRO-pcl-ros \
    ros-$ROS_DISTRO-perception-pcl && \
    pip install open3d scipy && \
    rm -rf /var/lib/apt/lists/*

COPY ./ros2 /ros2_ws/src
COPY ./docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash; cd ros2_ws && colcon build --symlink-install'

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]