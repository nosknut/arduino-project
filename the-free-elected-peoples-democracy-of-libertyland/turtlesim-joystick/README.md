## Hosting mqtt in docker

If you are on windows, open the ports in the firewall

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

## Run catkin commands with Docker

The following command says

- `docker exec`
  - Run the following command in a docker image
- `-it`
  - interactive shell session (don't run the container in the background)
- `ros:noetic-ros-base`
  - Use the ros:noetic-ros-base docker image
- `-v ./:/current-dir`
  - Attach the directory `./` in your computer to `/current-dir` inside the Docker container
- `-w /current-dir`
  - Open the interactive shell session at the current-dir we attached to our filesystem with `-v`
- `bash`
  - This is the command we told docker to run inside the specified work directory
  - `bash` will start a shell session where you can run normal commands

```
docker-compose up -it catkin
```

By running the command above you will open an interactive shell
session inside the ros:noetic-ros-base docker container, which is attached to
the directory in which you ran the command. Now you can run your
build commands, and the file changes will be reflected in your
actual filesystem. Once inside the Docker container, Run the
following commands to set up a catkin workspace.

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
