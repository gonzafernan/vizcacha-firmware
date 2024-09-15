#!/bin/bash

# Update micro-ROS submodule

ROS_DISTRO=humble

docker pull microros/micro_ros_static_library_builder:${ROS_DISTRO}
docker run -it --rm -v $(pwd):/project \
    --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library \
    microros/micro_ros_static_library_builder:${ROS_DISTRO}
