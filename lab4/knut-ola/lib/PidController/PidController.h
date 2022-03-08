#ifndef PidController_h
#define PidController_h
#include <Scaling.h>
#include <PidControllerConfig.h>
#include <Range.h>
#include <Arduino.h>

class PidController
{

public:
    PidControllerConfig pidConfig;

private:
    double area = 0;
    double lastError = 0;
    long previousUpdateTime = millis();
    double previousUpdateOutput = 0;

public:
    // Set kp, ki and/or kd to 0.0 to disable the respective mode
    PidController(PidControllerConfig pidConfig) : pidConfig(pidConfig)
    {
    }

    double update(double value, double target, bool clampOutput)
    {
        double error = value - target;

        long deltaTime = millis() - previousUpdateTime;
        if (deltaTime > pidConfig.updateTimeout)
        {
            // Ignore this update if it is too old
            previousUpdateTime = millis();
            return previousUpdateOutput;
        }

        double deltaError = error - lastError;

        double centerPointError = (error + lastError) / 2;
        area += centerPointError * deltaTime;

        float kp = pidConfig.kp;
        float ki = pidConfig.ki;
        float kd = pidConfig.kd;

        double output = kp * error + ki * area + kd * deltaError;

        lastError = error;
        previousUpdateTime = millis();

        int inputRangeWidth = pidConfig.inputRange.getWidth();
        double scaledOutput = Scaling::mapToRange(
            output,
            Range(-inputRangeWidth, inputRangeWidth),
            pidConfig.outputRange);

        previousUpdateOutput =
            clampOutput
                ? Scaling::clamp(scaledOutput, pidConfig.outputRange)
                : scaledOutput;

        return previousUpdateOutput;
    }
};

#endif
