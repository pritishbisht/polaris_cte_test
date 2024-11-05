# Use an official ROS Noetic base image
FROM ros:noetic-ros-core

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8

# Install necessary packages, including specified ROS packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-pip \
        python3-rosdep \
        build-essential \
        git \
        ros-noetic-gazebo-ros \
        ros-noetic-ackermann-msgs \
        ros-noetic-geometry2 \
        ros-noetic-hector-gazebo \
        ros-noetic-hector-models \
        ros-noetic-jsk-rviz-plugins \
        ros-noetic-ros-control \
        ros-noetic-ros-controllers \
        ros-noetic-velodyne-simulator && \
    rm -rf /var/lib/apt/lists/*

# Initialize rosdep and create Catkin workspace
RUN rosdep init && \
    rosdep update && \
    mkdir -p /root/gem_ws/src
WORKDIR /root/gem_ws

# Clone the POLARIS_GEM_e2 repository
RUN git clone https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2.git src/POLARIS_GEM_e2

# Install dependencies using rosdep in the workspace
RUN . /opt/ros/noetic/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y

# Build the Catkin workspace
RUN . /opt/ros/noetic/setup.sh && \
    catkin_make

# Source the workspace setup file after build
RUN echo "source /opt/ros/noetic/setup.sh" >> ~/.bashrc && \
    echo "source /root/gem_ws/devel/setup.bash" >> ~/.bashrc

# Set the default command to bash, so the container is ready for manual interaction
CMD ["bash"]