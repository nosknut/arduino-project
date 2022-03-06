#ifndef Scaling_h
#define Scaling_h
#include <Arduino.h>
namespace Scaling
{

    int clamp(const int value, const Range range)
    {
        if (value < range.minValue)
        {
            return range.minValue;
        }
        else if (value > range.maxValue)
        {
            return range.maxValue;
        }
        else
        {
            return value;
        }
    }

    int mapToRange(const int value, const Range inputRange, const Range outputRange)
    {
        return map(
            value,
            inputRange.minValue,
            inputRange.maxValue,
            outputRange.minValue,
            outputRange.maxValue);
    }

}

#endif
