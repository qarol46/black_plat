FROM ros:humble-ros-core

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV TZ=Europe/Moscow

ENV ROS_DISTRO=humble

RUN apt-get update && apt install -y python3-colcon-common-extensions \
    && apt-get install -y ros-$ROS_DISTRO-navigation2 \
    && apt-get install -y ros-$ROS_DISTRO-cartographer \
    && apt-get install -y ros-$ROS_DISTRO-cartographer-ros \
    && apt-get install -y ros-$ROS_DISTRO-tf2-ros \
    && apt-get install -y ros-$ROS_DISTRO-robot-localization \
    && apt-get install -y ros-$ROS_DISTRO-robot-state-publisher \
    && apt install -y ros-$ROS_DISTRO-perception-pcl \
  	&& apt install -y ros-$ROS_DISTRO-pcl-msgs \
    && apt install -y ros-$ROS_DISTRO-sensor-msgs \
    && apt install -y ros-$ROS_DISTRO-geometry-msgs \
    && apt install -y ros-$ROS_DISTRO-visualization-msgs \
    && rm -rf /var/lib/apt/lists/*


SHELL ["/bin/bash", "-c"]

COPY ./ros2 /ros2_ws/src
COPY ./docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash; cd ros2_ws && colcon build --symlink-install'

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]