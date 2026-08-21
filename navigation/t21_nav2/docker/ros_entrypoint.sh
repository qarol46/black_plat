#!/bin/bash
set -e

ROS_DISTRO="humble"

# setup ros2 environment
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
source "/opt/ros/$ROS_DISTRO/local_setup.bash"
source "/ros2_ws/install/setup.bash"


exec "$@"
