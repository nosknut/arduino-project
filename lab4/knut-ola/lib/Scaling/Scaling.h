#ifndef Scaling_h
#define Scaling_h
#include <Range.h>
namespace Scaling
{

    /**
     * Keeps the value within the given range. If it exceeds the range, the
     * return value will be the maximum or minimum value given by the range.
     *
     * @param value value to be clamped
     * @param range the range limiting the value
     * @return int the clamped value
     */
    int clamp(const int value, Range range)
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

    /** Returns the amount that was left out of a range
     * Example:
     * remainderFromClamp(-1, Range(0, 10)) == -1
     * remainderFromClamp(0, Range(0, 10)) == 0
     * remainderFromClamp(1, Range(0, 10)) == 0
     * remainderFromClamp(9, Range(0, 10)) == 0
     * remainderFromClamp(10, Range(0, 10)) == 0
     * remainderFromClamp(11, Range(0, 10)) == 1
     */
    int remainderFromClamp(const int value, Range range)
    {
        return value - clamp(value, range);
    }

    /**
     * @brief Same as in Arduino.h https://www.arduino.cc/reference/en/language/functions/math/map/
     */
    long map(long x, long in_min, long in_max, long out_min, long out_max)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    /**
     * @brief Same as the map() function but with ranges instead of many parameters
     *
     * @param value
     * @param inputRange
     * @param outputRange
     * @return long
     */
    long mapToRange(const long value, Range inputRange, Range outputRange)
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
