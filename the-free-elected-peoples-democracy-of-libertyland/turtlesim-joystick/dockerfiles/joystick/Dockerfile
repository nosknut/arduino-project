FROM ros:noetic-ros-base

WORKDIR .

COPY . .

RUN ls

RUN apt-get update

# RUN apt-get install -y git-all

# RUN git clone git@github.com:groove-x/mqtt_bridge.git
# RUN cd mqtt_bridge && python3 setup.py
COPY mqtt-bridge-configs/requirements.txt /
RUN pip3 install -r /mqtt-bridge-configs/requirements.txt
# RUN cd /mqtt-bridge-configs && pip3 install -r requirements.txt

RUN apt-get install -y ros-${ROS_DISTRO}-rosbridge-library
RUN apt-get install -y ros-${ROS_DISTRO}-mqtt-bridge
RUN apt-get install -y mosquitto mosquitto-clients
RUN apt-get install -y python3-pip
RUN rm -rf /var/lib/apt/lists/*

# RUN cd mqtt_bridge && catkin_make

RUN echo source "/opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
