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

}

#endif
