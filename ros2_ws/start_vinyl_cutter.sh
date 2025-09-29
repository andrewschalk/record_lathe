#!/bin/bash

# Source ROS 2
source /opt/ros/kilted/setup.bash

# Source workspace
source /home/ros/ros2_ws/install/setup.bash

# Launch the node
ros2 launch /home/ros/ros2_ws/launch/vinyl_cutter_launch.py
