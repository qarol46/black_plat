# Use the official ROS Noetic desktop full image
FROM osrf/ros:noetic-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# 1. Install system dependencies and Python tools 
# Note: Most ROS tools are already in the base image, but we ensure pip and build-essentials exist
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    vim \
    python3-pip \
    python3-catkin-tools \
    openssh-server \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# 2. Custom rosdep setup 
# RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list
# RUN sudo rosdep init 

# 3. Install Python requirements
RUN pip3 install --no-cache-dir scipy numpy matplotlib rosbags

# Setup environment sourcing for the user
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc

# 4. Build the workspace
# We use /bin/bash -c to ensure the setup.bash is sourced for the build command
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash"
WORKDIR /catkin_ws

# Clean up build/devel if you only want the image for running (as per your original script)
# Note: Usually you want to keep these to actually use the workspace!
# RUN rm -rf build devel
CMD ["bash"]