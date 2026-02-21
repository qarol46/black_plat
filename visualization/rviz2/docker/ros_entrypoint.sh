#!/bin/bash
set -e

ROS_DISTRO="humble"

# setup ros2 environment
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
source "/opt/ros/$ROS_DISTRO/local_setup.bash"


exec "$@"
