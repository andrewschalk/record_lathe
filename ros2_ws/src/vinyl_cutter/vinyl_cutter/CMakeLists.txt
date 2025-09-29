cmake_minimum_required(VERSION 3.8)
project(vinyl_cutter)

# Generate interfaces from .srv files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlayAudio.srv"
)