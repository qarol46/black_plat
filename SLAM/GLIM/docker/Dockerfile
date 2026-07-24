ARG BASE_IMAGE=koide3/gtsam_points:jammy_gtsam4.3a0
FROM $BASE_IMAGE

ARG BUILD_WITH_CUDA=OFF

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV TZ=Europe/Moscow

ENV ROS_DISTRO=humble

# RUN apt-get update \
#   && apt-get upgrade -y \
#   # install dependencies
#   && apt-get install --no-install-recommends -y \
#     curl libfmt-dev libspdlog-dev libopencv-dev zenity \
#   # install ROS
#   && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
#   && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
#   && apt-get update \
#   && apt-get install --no-install-recommends -y \
#     python3-colcon-common-extensions \
#     ros-$ROS_DISTRO-ros-base ros-$ROS_DISTRO-cv-bridge \
#     ros-$ROS_DISTRO-image-transport-plugins ros-$ROS_DISTRO-tf2-ros \

RUN apt-get update \
  && apt-get install --no-install-recommends -y curl ca-certificates gpg-agent \
  && update-ca-certificates \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && apt-get update \
  && apt-get install --no-install-recommends -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-${ROS_DISTRO}-ros-base ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-image-transport-plugins ros-$ROS_DISTRO-tf2-ros \
    libfmt-dev libspdlog-dev libopencv-dev \
    libboost-all-dev libglfw3-dev libmetis-dev \
    software-properties-common \
    git cmake build-essential wget unzip libboost-all-dev \
    libmetis-dev libomp-dev \
    libpng-dev libjpeg-dev libeigen3-dev libglm-dev libglfw3-dev \
    libwebkit2gtk-4.0-37 libjavascriptcoregtk-4.0-18 libgtk-3-0 \
# install gtsam
  # && git clone https://github.com/borglab/gtsam \
  # &&  cd gtsam && git checkout 4.3a0 \
  # && mkdir build \
  # && cd build \
  # && cmake .. -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  #             -DGTSAM_BUILD_TESTS=OFF \
  #             -DGTSAM_WITH_TBB=OFF \
  #             -DGTSAM_USE_SYSTEM_EIGEN=ON \
  #             -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  # && make -j$(nproc) \
  # && sudo make install \
# Install Iridescence for visualization
# This is optional but highly recommended
  && git clone https://github.com/koide3/iridescence --recursive \
  && mkdir iridescence/build && cd iridescence/build \
  && cmake .. -DCMAKE_BUILD_TYPE=Release \
  && make -j$(nproc) \
  && sudo make install \

# Install gtsam_points
  # && git clone https://github.com/koide3/gtsam_points \
  # && mkdir gtsam_points/build && cd gtsam_points/build \
  # && cmake .. -DBUILD_WITH_CUDA=${BUILD_WITH_CUDA} \
  # && make -j$(nproc) \
  # && sudo make install \
# After installation, make shared libraries visible to the system
  && sudo ldconfig \
# clean
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/* 

COPY ./ros2 /ros2_ws/src
COPY ./docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash; cd /ros2_ws && colcon build --cmake-args \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_CUDA=${BUILD_WITH_CUDA} -DBUILD_WITH_VIEWER=ON'
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
