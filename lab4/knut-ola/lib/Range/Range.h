struct Range
{
    const int minValue;
    const int maxValue;
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
