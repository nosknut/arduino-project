## Starting the entire stack
Run ```ipconfig``` in a terminal to find your ip address, and update the
existing ip address at the following path in [docker-compose.yml](docker-compose.yml)
```
services:
  turtlesim:
    environment:
      - DISPLAY=your_local_ip_address:0.0
```
On Windows
- Install [VcXsrv Windows X Server Download](https://sourceforge.net/projects/vcxsrv/) and double click [config.xlaunch](config.xlaunch) while in file explorer to run it. You can open file explorer by typing ```start .``` (include the period) in your terminal

Now run the following commands in order. After running the last one, you should see a blue window with a turtle a few seconds after all the docker images have started.
```
docker-compose run install-node-red
docker-compose up -d master mqtt mqtt-ros-bridge rosserial differential-drive node-red turtlesim
```
Node-RED will be hosted on http://localhost:1880 and the Node-RED UI will be hosted on http://localhost:1880/ui

You can now control the blue turtlesim simulation window with the Node-RED website.

Run the following command to shut everything down
```
docker-compose down
```
---

## Hosting mqtt in docker

If you are on windows, open the ports in the firewall
   * NB! The following commands must be run in ```powershell```

Port 1883

```
New-NetFirewallRule -DisplayName "ALLOW TCP PORT 1883" -Direction inbound -Profile Any -Action Allow -LocalPort 1883 -Protocol TCP
```

Port 1901

```
New-NetFirewallRule -DisplayName "ALLOW TCP PORT 1901" -Direction inbound -Profile Any -Action Allow -LocalPort 1901 -Protocol TCP
```

Start the docker container

```
docker-compose up -d mqtt
```

Shut down the docker container so stop hosting mqtt

```
docker-compose stop mqtt
```

## Run catkin commands with Docker
Navigate to the folder that contains this [docker-compose.yml](./docker-compose.yml) file

It contains a service called "catkin". You will use this service to run catkin commands in your current directory.

Run the following command to start and enter the catkin docker container.
```
docker-compose run catkin
```

By running the command above you will open an interactive shell
session inside the catkin docker-compose service, which is attached to
the directory in which you ran the command. Now you can run your
build commands, and the file changes will be reflected in your
actual filesystem. Once inside the Docker container, Run the
following commands to set up a catkin workspace.

NB! The following steps can be skipped.

```
mkdir test-package
mkdir test-package/src
cd test-package/src
catkin_init_workspace
```

Now create a package with the `roscpp` and `std_msgs` dependencies

```
catkin_create_pkg test-package roscpp std_msgs
```
Enter the following command to list the files that have been created.
```
ls
```

If you look at the directory from which you started the catkin docker-compose service, you will see the new folder, containing the newly created package files.

The catkin service uses the ```ros:noetic-ros-base``` docker image. This means all of the commands and features in this image will be available to you. In other words, this container will not only run catkin, but most ROS commands.

## ROS Setup

Useful links

- [Docker Desktop](https://www.docker.com/products/docker-desktop/)
- [Installing Docker on Raspberry PI](https://www.jfrog.com/connect/post/install-docker-compose-on-raspberry-pi/)

### Starting the master ros node docker container

```
docker-compose up -d master
```

You can see the container running with the following command

```
docker ps
```

### Starting the turtlesim ros node docker container

1. #### Running GUI applications in docker containers on windows
   - This is used for the turtlesim container [Tutorial](https://jack-kawell.com/2019/09/11/setting-up-ros-in-windows-through-docker/)
   - [VcXsrv Windows X Server Download](https://sourceforge.net/projects/vcxsrv/)
2. Click the [config.xlaunch](config.xlaunch) file to start the display client

3. Update [docker-compose.yml](docker-compose.yml) to include your computers local ip-address
   - The ip address can not be localhost or 127.0.0.1
   - The block below shows the path for the setting that must be change

```
services:
  turtlesim:
    environment:
      - DISPLAY=<your_ip_address>:0.0
```

After you have set your IP Address in the [docker-compose.yml](docker-compose.yml) file, start the turtlesim docker container

```
docker-compose up turtlesim
```

### Rebuilding the turtlesim docker image

- Use this if you made changes to the turtlesim [Dockerfile](dockerfiles/turtlesim/Dockerfile)

```
docker-compose up --build turtlesim
```

### Running commands in an active docker container

```
docker exec -it <container_name> bash
source ros_entrypoint.sh
rosrun <package_name> <package_node>
```

### Shutting down all docker containers

```
docker-compose down
```

### Shutting down a specific docker container

```
docker-compose down <container_name>
```

## Controling turtlesim with keyboard

[Video Tutorial](https://www.youtube.com/watch?v=PlS6YCu5CT4)

Start docker containers

```
docker-compose up -d master turtlesim
```

Enter the turtlesim container and run the control package

```
docker exec -it turtlesim bash
rosrun turtlesim turtle_teleop_key
```

Open another terminal to see your command messages

```
docker exec -it turtlesim bash
rostopic list
rostopic echo turtle1/cmd_vel
```

To shut everything down, close the terminals and run the following code to stop the docker containers that are running in the background

```
docker-compose down
```
