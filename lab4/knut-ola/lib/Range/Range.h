#ifndef Range_h
#define Range_h

struct Range
{
    int minValue;
    int maxValue;
    Range(int minValue, int maxValue) : minValue(minValue), maxValue(maxValue)
    {
    }

    int getMidValue() const
    {
        return (minValue + maxValue) / 2;
    }

    int getWidth() const
    {
        return maxValue - minValue;
    }
};

#endif
