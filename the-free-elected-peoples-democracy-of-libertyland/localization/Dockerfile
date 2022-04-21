FROM ros:noetic-ros-base

# install ros package
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-rviz && \
    ros-${ROS_DISTRO}-turtlesim && \
    rm -rf /var/lib/apt/lists/*

# launch ros package
CMD ["roslaunch", "turtlesim", "turtlesim_node"]