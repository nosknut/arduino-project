#ifndef PidController_h
#define PidController_h
#include <Scaling.h>
#include <Arduino.h>

class PidController
{
public:
    float kp = 0;
    float ki = 0;
    float kd = 0;

private:
    // Max amount of time between updates for the update to be valid
    long updateTimeout = 20;
    const Range inputRange;
    const Range outputRange;

    float area = 0;
    float lastError = 0;
    long previousUpdateTime = millis();
    float previousUpdateOutput = 0;

public:
    // Set kp, ki and/or kd to 0.0 to disable the respective mode
    PidController(float kp, float ki, float kd, const Range inputRange, const Range outputRange)
        : kp(kp),
          ki(ki),
          kd(kd),
          inputRange(inputRange),
          outputRange(outputRange)
    {
    }

    float update(float value, float target)
    {
        float error = value - target;
        float deltaTime = millis() - previousUpdateTime;
        if (deltaTime > updateTimeout)
        {
            // Ignore this update if it is too old
            previousUpdateTime = millis();
            return previousUpdateOutput;
        }
        float deltaError = error - lastError;
        float centerPointError = (error + lastError) / 2;
        area += centerPointError * deltaTime;
        float output = kp * error + ki * area + kd * deltaError;
        lastError = error;
        previousUpdateTime = millis();

        previousUpdateOutput = Scaling::mapToRange(
            Scaling::clamp(output, inputRange),
            inputRange,
            outputRange);

        return previousUpdateOutput;
    }
};

#endif
