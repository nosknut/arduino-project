#ifndef Scaling_h
#define Scaling_h
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

    // Returns the amount that was left out of a range
    // Example:
    // remainderFromClamp(-1, Range(0, 10)) == -1
    // remainderFromClamp(0, Range(0, 10)) == 0
    // remainderFromClamp(1, Range(0, 10)) == 0
    // remainderFromClamp(9, Range(0, 10)) == 0
    // remainderFromClamp(10, Range(0, 10)) == 0
    // remainderFromClamp(11, Range(0, 10)) == 1
    int remainderFromClamp(const int value, const Range range)
    {
        return value - clamp(value, range);
    }

    // Arduino.h
    long map(long x, long in_min, long in_max, long out_min, long out_max)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    long mapToRange(const long value, const Range inputRange, const Range outputRange)
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
