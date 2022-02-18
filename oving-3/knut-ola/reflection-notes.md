## Files
- [platformio.ini](platformio.ini)
- [main.cpp](src/main.cpp)
- [lib](lib)
    - [ApplicationConfig.h](lib/ApplicationConfig/ApplicationConfig.h)
    - [PlayerConfig.h](lib/PlayerConfig/PlayerConfig.h)
    - [Range.h](lib/Range/Range.h)
    - [RgbLedConfig.h](lib/RgbLedConfig/RgbLedConfig.h)
    - [Timer.h](lib/Timer/Timer.h)

## Reflection notes
For this exercise i decided to adopt a nested sequenctial architecture.
The benefits of this should be evident when looking at the code. It is very
easy to follow and can be taken at face value, with each step leading into
another function, thus isolating each task to its own scope.
The backside with this architecture is that it creates a blocking sequence, preventing
any form of continous updates without manually doing such operations during any of
the given while-loops. While this remains an interresting tangent, i do not believe i will
adopt this architecture in any of my future projects.