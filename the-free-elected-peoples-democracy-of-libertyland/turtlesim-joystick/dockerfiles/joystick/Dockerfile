FROM ros:noetic-ros-base

RUN apt-get update

RUN apt-get install -y git

RUN git clone git@github.com:groove-x/mqtt_bridge.git
RUN cd mqtt_bridge && python3 setup.py

RUN apt-get install -y python3-pip
RUN apt-get install -y ros-${ROS_DISTRO}-rosbridge-library
RUN apt-get install -y mosquitto mosquitto-clients
RUN rm -rf /var/lib/apt/lists/*

RUN cd mqtt_bridge && catkin_make

RUN echo source "/opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
