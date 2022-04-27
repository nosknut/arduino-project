## ROS Setup

Useful links
* [Docker Desktop](https://www.docker.com/products/docker-desktop/)
* [Installing Docker on Raspberry PI](https://www.jfrog.com/connect/post/install-docker-compose-on-raspberry-pi/)

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
   * This is used for the turtlesim container [Tutorial](https://jack-kawell.com/2019/09/11/setting-up-ros-in-windows-through-docker/)
   * [VcXsrv Windows X Server Download](https://sourceforge.net/projects/vcxsrv/)
2. Click the [config.xlaunch](config.xlaunch) file to start the display client

2. Update [docker-compose.yml](docker-compose.yml) to include your computers local ip-address
   * The ip address can not be localhost or 127.0.0.1
   * The block below shows the path for the setting that must be change
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