#ifndef PidController_h
#define PidController_h
#include <Arduino.h>
#include <Scaling.h>

class PidController
{
private:
    // Max amount of time between updates for the update to be valid
    long updateTimeout = 2;
    double kp = 0;
    double ki = 0;
    double kd = 0;
    const Range inputRange;
    const Range outputRange;

    double area = 0;
    double lastError = 0;
    long lastTime = millis();
    double lastOutput = 0;

public:
    PidController(double kp, double ki, double kd, const Range inputRange, const Range outputRange)
        : kp(kp),
          ki(ki),
          kd(kd),
          inputRange(inputRange),
          outputRange(outputRange)
    {
    }

    double update(double value, double target)
    {
        double error = target - value;
        double deltaTime = millis() - lastTime;
        if (deltaTime > updateTimeout)
        {
            // Ignore this update if it is too old
            lastTime = millis();
            return lastOutput;
        }
        double deltaError = error - lastError;
        double centerPointError = (error + lastError) / 2;
        area += centerPointError * deltaTime;
        double output = kp * error + ki * area + kd * deltaError;
        lastError = error;
        lastTime = millis();

        lastOutput = map(
            Scaling::clamp(output, inputRange),
            inputRange.minValue,
            inputRange.maxValue,
            outputRange.minValue,
            outputRange.maxValue);

        return lastOutput;
    }
};

#endif
