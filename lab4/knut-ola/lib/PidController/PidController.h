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
    /**
     * @brief Set kp, ki and/or kd to 0.0 to disable the respective mode
     *
     * @param pidConfig
     */
    PidController(PidControllerConfig pidConfig) : pidConfig(pidConfig)
    {
    }

    /**
     * NB!
     * For I, PI, ID og PID (anything with I) regulators, the input value MUST be allowed
     * to exceed the input range. For example, if the input range is [0, 100], the "value" argument
     * must be allowed to be greater than 100, and less than 0. If this is not done, the regulator
     * will be unable to calculate the correct output, if the target value is ever set to 100 or 0.
     * It will also get less responsive as the target value approaches the limits of the inputRange.
     * In short: Do not restrict the input value to the input range, unless you want
     * your watertank to overflow.
     *
     * @param value the sensor reading
     * @param target the target sensor reading
     * @param clampOutput true if the output should always stay
     * within the output range specified in the config
     * @return the output value
     */
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
