#ifndef Photoresistor_h
#define Photoresistor_h

#include <Arduino.h>
#include <Range.h>

class Photoresistor
{
    // Start with oposite values
    Range limits = Range(1023, 0);
    const int pin;

public:
    Photoresistor(const int pin) : pin(pin)
    {
    }

    int mapWithinLimits(const int value)
    {
        return map(value, limits.minValue, limits.maxValue, 0, 1023);
    }
    int read()
    {
        const int value = analogRead(pin);
        // Always update the limits
        if (value < limits.minValue)
        {
            limits.minValue = value;
        }
        if (value > limits.maxValue)
        {
            limits.maxValue = value;
        }
        return mapWithinLimits(value);
    }

    void setup()
    {
        pinMode(pin, INPUT);
    }
};

#endif
