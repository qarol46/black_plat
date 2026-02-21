FROM t21-rviz2


ENV DEBIAN_FRONTEND=noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV TZ=Europe/Moscow

ENV ROS_DISTRO=humble

RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-rtabmap \
    ros-$ROS_DISTRO-rtabmap-ros \
    && rm -rf /var/lib/apt/lists/

SHELL ["/bin/bash", "-c"]

COPY ./ros2 /ros2_ws/src
COPY ./docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash; cd ros2_ws && colcon build --symlink-install'

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]