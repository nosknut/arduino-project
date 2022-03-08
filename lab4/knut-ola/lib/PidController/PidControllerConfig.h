#ifndef PidControllerConfig_h
#define PidControllerConfig_h
#include <Scaling.h>
#include <Range.h>

struct PidControllerConfig
{
    float kp = 0;
    float ki = 0;
    float kd = 0;
    Range inputRange;
    Range outputRange;
    // Max amount of time between updates for the update to be valid
    long updateTimeout = 20;

    PidControllerConfig(
        float kp,
        float ki,
        float kd,
        Range inputRange,
        Range outputRange)
        : kp(kp),
          ki(ki),
          kd(kd),
          inputRange(inputRange),
          outputRange(outputRange)
    {
    }
};

#endif
