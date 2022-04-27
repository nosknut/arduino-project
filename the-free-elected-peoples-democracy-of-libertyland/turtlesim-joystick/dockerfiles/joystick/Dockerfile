FROM ros:noetic-ros-base

RUN apt-get update

# RUN apt-get install -y python3-pip
RUN apt-get install -y ros-${ROS_DISTRO}-rosbridge-library
RUN apt-get install -y ros-${ROS_DISTRO}-mqtt-bridge
RUN apt-get install -y mosquitto mosquitto-clients

RUN rm -rf /var/lib/apt/lists/*

# Copy configs into correct folder
RUN  cd /opt/ros/noetic/share/mqtt_bridge/config/config && cp -a -r * ../

# RUN pip3 install -r requirements.txt

RUN echo source "/opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
