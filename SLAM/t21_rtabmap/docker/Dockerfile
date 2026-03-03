FROM t21-rviz2

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV TZ=Europe/Moscow

ENV ROS_DISTRO=humble

RUN apt-get update \
    && apt-get install -y git \
    python3-rosdep \
    libsqlite3-dev \
    liboctomap-dev \
    libpcl-dev \
    libproj-dev \
    ros-$ROS_DISTRO-nav2-costmap-2d \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-octomap-msgs \
    ros-$ROS_DISTRO-pcl-conversions \
    ros-$ROS_DISTRO-pcl-ros \
    ros-$ROS_DISTRO-grid-map-ros \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-libg2o \
    ros-$ROS_DISTRO-gtsam \
    ros-$ROS_DISTRO-libpointmatcher \
    ros-$ROS_DISTRO-qt-gui-cpp \
    ros-$ROS_DISTRO-image-geometry \
    ros-$ROS_DISTRO-apriltag-msgs \
    ros-$ROS_DISTRO-aruco-msgs \
    ros-$ROS_DISTRO-aruco-opencv-msgs \
    ros-$ROS_DISTRO-imu-filter-madgwick \
    ros-$ROS_DISTRO-realsense2-camera \
    ros-$ROS_DISTRO-velodyne \
    ros-$ROS_DISTRO-tf2-eigen && \
    rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]

COPY ./ros2 /ros2_ws/src
COPY ./docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
WORKDIR /ros2_ws
RUN sudo rosdep init
RUN git clone https://github.com/introlab/rtabmap.git src/rtabmap
RUN git clone --branch ros2 https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros
RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y
RUN export MAKEFLAGS="-j6"
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash; colcon build --symlink-install'
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]