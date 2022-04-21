FROM osrf/ros:kinetic-desktop-full

# install ros package
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-turtlesim && \
    rm -rf /var/lib/apt/lists/*

RUN echo source "/opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo 'export DISPLAY=192.168.1.190:0.0' >> ~/.bashrc
