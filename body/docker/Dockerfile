FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV TZ=Europe/Moscow

ENV ROS_DISTRO=humble

# Realsense installation
# Create the keyrings directory
RUN mkdir -p /etc/apt/keyrings

# Download and add the RealSense signing key
RUN curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc | \
    gpg --dearmor | tee /etc/apt/keyrings/librealsenseai.gpg > /dev/null

# Install apt-transport-https to allow APT to use HTTPS repositories
RUN apt-get update && apt-get install -y apt-transport-https

# Add the RealSense repository to APT sources
RUN echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo `lsb_release -cs` main" | \
    tee /etc/apt/sources.list.d/librealsense.list

# Update the package list with the new repository
RUN apt-get update

# Install the RealSense userspace utilities (optional, but useful for debugging)
RUN apt-get install -y librealsense2-utils

# REMOVED: librealsense2-dkms - not needed as ROS2 packages handle this

# Install base dependencies
RUN apt-get install -y \
    build-essential \
    cmake \
    python3 \
    python3-pip \
    python3-colcon-common-extensions \
    libglew-dev \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-control-msgs \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-controller-interface \
    ros-$ROS_DISTRO-controller-manager-msgs \
    ros-$ROS_DISTRO-diff-drive-controller \
    ros-$ROS_DISTRO-hardware-interface \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-joint-limits \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-joint-state-broadcaster \
    ros-$ROS_DISTRO-joint-trajectory-controller \
    ros-$ROS_DISTRO-admittance-controller \
    ros-$ROS_DISTRO-twist-mux \
    ros-$ROS_DISTRO-twist-mux-msgs \
    #ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-librealsense2 \
    ros-$ROS_DISTRO-realsense2-camera \
    ros-$ROS_DISTRO-realsense2-camera-msgs \
    ros-$ROS_DISTRO-realsense2-description \
    ros-$ROS_DISTRO-camera-calibration-parsers \
    ros-$ROS_DISTRO-camera-info-manager \
    ros-$ROS_DISTRO-laser-geometry \
    ros-$ROS_DISTRO-velodyne \
    ros-$ROS_DISTRO-velodyne-driver \
    ros-$ROS_DISTRO-velodyne-msgs \
    ros-$ROS_DISTRO-velodyne-pointcloud \
    ros-$ROS_DISTRO-point-cloud-interfaces \
    ros-$ROS_DISTRO-point-cloud-msg-wrapper \
    ros-$ROS_DISTRO-pcl-conversions \
    ros-$ROS_DISTRO-pcl-msgs \
    ros-$ROS_DISTRO-pcl-ros \
    ros-$ROS_DISTRO-perception-pcl && \
    rm -rf /var/lib/apt/lists/*

# Xsens installation would go here
# TODO

COPY ./ros2 /ros2_ws/src
COPY ./docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash; cd ros2_ws && colcon build --symlink-install'

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]