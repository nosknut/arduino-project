FROM ros:noetic-ros-base

RUN apt-get update

RUN apt-get install -y ros-${ROS_DISTRO}-move-base

RUN rm -rf /var/lib/apt/lists/*

# RUN cd mqtt_bridge && catkin_make

RUN echo source "/opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
