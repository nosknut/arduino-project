FROM osrf/ros:kinetic-desktop-full

RUN apt-get update

RUN sudo apt-get install -y ros-${ROS_DISTRO}-rqt-reconfigure

RUN rm -rf /var/lib/apt/lists/*

RUN echo source "/opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
