FROM ubuntu

RUN apt-get update

RUN apt-get install -y ros-kinetic-rviz

RUN rm -rf /var/lib/apt/lists/*

RUN echo source "/opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
