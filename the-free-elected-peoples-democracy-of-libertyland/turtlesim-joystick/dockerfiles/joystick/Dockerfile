FROM ros:noetic-ros-base

RUN apt-get update

COPY mqtt-bridge-configs /mqtt-bridge-configs
RUN apt-get install -y python3-pip
RUN pip3 install -r /mqtt-bridge-configs/dev-requirements.txt
RUN rm -rf /mqtt-bridge-configs

RUN apt-get install -y ros-${ROS_DISTRO}-rosbridge-library
RUN apt-get install -y ros-${ROS_DISTRO}-mqtt-bridge
RUN apt-get install -y mosquitto mosquitto-clients
RUN rm -rf /var/lib/apt/lists/*

# RUN cd mqtt_bridge && catkin_make

RUN echo source "/opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
