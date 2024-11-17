# Use the official ROS Noetic desktop full image as the base
FROM osrf/ros:noetic-desktop-full

# Set build arguments
ARG DEBIAN_FRONTEND=noninteractive

# Use bash as the default shell
SHELL ["/bin/bash", "-c"]

# Install necessary packages
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    build-essential \
    cmake \
    sudo \
    ros-noetic-slam-gmapping \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-turtlebot3-* \
    x11-xserver-utils \
    mesa-utils \
    libgl1-mesa-glx \
    libegl1-mesa \
    && rm -rf /var/lib/apt/lists/*

# Set environment variables for GUI support
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1

# Set the environment variable for TurtleBot3
ENV TURTLEBOT3_MODEL=burger

# Create workspace directories
WORKDIR /home/ros_ws/catkin_ws/src
RUN mkdir -p /home/ros_ws/catkin_ws/src

# Initialize the workspace
WORKDIR /home/ros_ws/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Source ROS setup files on container startup
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && bash"]
