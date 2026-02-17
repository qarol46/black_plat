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
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-pointcloud-to-laserscan \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-nav-2d-msgs \
    ros-$ROS_DISTRO-nav-2d-utils \
    ros-$ROS_DISTRO-nav-msgs \
    ros-$ROS_DISTRO-nav2-amcl \
    ros-$ROS_DISTRO-nav2-behavior-tree \
    ros-$ROS_DISTRO-nav2-behaviors \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-nav2-bt-navigator \
    ros-$ROS_DISTRO-nav2-collision-monitor \
    ros-$ROS_DISTRO-nav2-common \
    ros-$ROS_DISTRO-nav2-constrained-smoother \
    ros-$ROS_DISTRO-nav2-controller \
    ros-$ROS_DISTRO-nav2-core \
    ros-$ROS_DISTRO-nav2-costmap-2d \
    ros-$ROS_DISTRO-nav2-dwb-controller \
    ros-$ROS_DISTRO-nav2-lifecycle-manager \
    ros-$ROS_DISTRO-nav2-map-server \
    ros-$ROS_DISTRO-nav2-mppi-controller \
    ros-$ROS_DISTRO-nav2-msgs \
    ros-$ROS_DISTRO-nav2-navfn-planner \
    ros-$ROS_DISTRO-nav2-planner \
    ros-$ROS_DISTRO-nav2-regulated-pure-pursuit-controller \
    ros-$ROS_DISTRO-nav2-rotation-shim-controller \
    ros-$ROS_DISTRO-nav2-route \
    ros-$ROS_DISTRO-nav2-rviz-plugins \
    ros-$ROS_DISTRO-nav2-simple-commander \
    ros-$ROS_DISTRO-nav2-smac-planner \
    ros-$ROS_DISTRO-nav2-smoother \
    ros-$ROS_DISTRO-nav2-theta-star-planner \
    ros-$ROS_DISTRO-nav2-util \
    ros-$ROS_DISTRO-nav2-velocity-smoother \
    ros-$ROS_DISTRO-nav2-voxel-grid \
    ros-$ROS_DISTRO-nav2-waypoint-follower \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-pointcloud-to-laserscan \
    ros-$ROS_DISTRO-pcl-msgs \
    


COPY ./ros2 /ros2_ws/src
COPY ./docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash; cd ros2_ws && colcon build --symlink-install'

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
