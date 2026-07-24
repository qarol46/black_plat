  #!/bin/bash
  . /opt/ros/humble/setup.bash
  . /ros2_ws/install/setup.bash
  ros2 run glim_ros glim_rosnode --ros-args -p config_path:=/glim/config