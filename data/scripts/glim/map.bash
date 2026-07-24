#!/bin/bash
. /opt/ros/humble/setup.bash
. /ros2_ws/install/setup.bash

/ros2_ws/install/glim_ros/lib/glim_ros/offline_viewer \
  --config_path /glim/config \
  "$@"