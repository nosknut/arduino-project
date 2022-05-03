FROM ros:noetic-ros-base

RUN apt-get update

RUN apt-get install -y ros-${ROS_DISTRO}-rosbridge-suite

RUN rm -rf /var/lib/apt/lists/*

RUN echo source "/opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
