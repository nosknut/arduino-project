#ifndef ServoConfig_h
#define ServoConfig_h

#include <Range.h>

struct ServoConfig
{
    const int servoPin;
    const Range angleLimits;
};

#endif
