FROM ros:kilted-ros-core

# Install git, colcon, rosdep, and GUI deps (X11 + Qt fixes)
RUN apt-get update && apt-get install -y \
    git \
    python3-rosdep \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    libgl1 \
    libxext6 \
    libxrender1 \
    libxcb-xinerama0 \
 && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y wget

# Setup rosdep (if not already in base image)
RUN rosdep init 2>/dev/null || true && rosdep update

# Workspace directory (mounted at runtime)
WORKDIR /root/ros2_ws

# Auto-source ROS + workspace
RUN echo "source /opt/ros/kilted/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /root/ros2_ws/install/setup.bash ]; then source /root/ros2_ws/install/setup.bash; fi" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=101" >> /root/.bashrc

# Install turtlesim (GUI demo)
RUN apt-get update && apt-get install -y ros-kilted-turtlesim && rm -rf /var/lib/apt/lists/*

# Connect to x server
RUN echo "export DISPLAY=host.docker.internal:0.0" >> /root/.bashrc

# Install rqt
RUN apt-get update && apt-get install -y ros-kilted-rqt* && rm -rf /var/lib/apt/lists/*

# Install some needed packages
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-rviz2 \
 && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y python3-pip
RUN pip3 install --no-cache-dir --break-system-packages flask

# Expose Flask default port
EXPOSE 5000