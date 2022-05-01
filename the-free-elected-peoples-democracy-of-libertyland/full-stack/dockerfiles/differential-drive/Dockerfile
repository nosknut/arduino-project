FROM ros:noetic-ros-base

RUN apt-get -y update
# needed because the scripts use python instead of python3
RUN sudo apt install python-is-python3
RUN rm -rf /var/lib/apt/lists/*

RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && catkin_make
COPY ./dockerfiles/differential-drive/differential-drive ./src/differential-drive 
RUN . devel/setup.sh && catkin_make


ENTRYPOINT /bin/bash -c "source devel/setup.bash && roslaunch /differential-drive-configs/differential_drive.launch"
