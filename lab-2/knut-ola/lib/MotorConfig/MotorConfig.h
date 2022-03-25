#ifndef MotorConfig_h
#define MotorConfig_h

#include <Range.h>

struct MotorConfig
{
    const int enablePin;
    const int in1Pin;
    const int in2Pin;
    const Range speedLimits;
};

#endif
