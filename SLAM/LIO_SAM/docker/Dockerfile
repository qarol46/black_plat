FROM t21-rviz2

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV TZ=Europe/Moscow

ENV ROS_DISTRO=humble

RUN apt-get update \
    && apt-get install -y curl \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && apt-get update \
    && apt-get install -y ros-$ROS_DISTRO-slam-toolbox \
    && apt-get install -y ros-$ROS_DISTRO-pointcloud-to-laserscan \
    && apt-get install -y ros-$ROS_DISTRO-robot-localization \
    && apt-get install -y ros-$ROS_DISTRO-tf2-ros \ 
    && apt install -y ros-$ROS_DISTRO-perception-pcl \
  	&& apt install -y ros-$ROS_DISTRO-pcl-msgs \
  	&& apt install -y ros-$ROS_DISTRO-vision-opencv \
    && rm -rf /var/lib/apt/lists/*
# delete tf2-ros
RUN apt-get update \
    && apt install -y software-properties-common \
    && add-apt-repository -y ppa:borglab/gtsam-release-4.1 \
    && apt-get update \
    && apt install -y libgtsam-dev libgtsam-unstable-dev \
    && rm -rf /var/lib/apt/lists/*

# SHELL ["/bin/bash", "-c"]

COPY ./ros2 /ros2_ws/src
COPY ./docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash; cd ros2_ws && colcon build --symlink-install'

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]