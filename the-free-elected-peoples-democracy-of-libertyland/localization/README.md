## ROS Setup
Starting docker container
```
docker-compose up -d
```

Rebuilding docker image
- Use this if you made changes to [Dockerfile](Dockerfile)
```
docker-compose up -d --build
```

Running commands in an active docker container
```
docker exec -it localization_talker_1 /bin/bash
source ros_entrypoint.sh
rosrun turtlesim turtlesim_node
```

Running GUI applications in docker containers on windows

* [Tutorial](https://jack-kawell.com/2019/09/11/setting-up-ros-in-windows-through-docker/)
* [VcXsrv Windows X Server Download](https://sourceforge.net/projects/vcxsrv/)


## Documentation
https://nosknut.github.io/arduino-project/annotated.html

## Setup
- Install python
    - [Download](https://www.python.org/downloads/)
    - [Tutorial](https://realpython.com/installing-python/)
- Install [PlatformIO](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide)
    - [Tutorial for setup and use](https://youtu.be/JmvMvIphMnY)

## Testing
- Install GCC
    - https://www.youtube.com/watch?v=8Ib7nwc33uA&ab_channel=DerekBanas
    - Select Environment to "native"
    - "Start Debugging" through the PlatformIO extension