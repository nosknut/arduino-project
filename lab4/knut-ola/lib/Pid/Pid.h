#ifndef PidController_h
#define PidController_h
#include <Arduino.h>

class PidController
{
private:
    double kp = 0;
    double ki = 0;
    double kd = 0;

    double area = 0;
    double lastError = 0;
    long lastTime = millis();

public:
    PidController(double kp, double ki, double kd)
        : kp(kp),
          ki(ki),
          kd(kd)
    {
    }

    double update(double value, double target)
    {
        double error = target - value;
        double deltaTime = millis() - lastTime;
        double deltaError = error - lastError;
        double centerPointError = (error + lastError) / 2;
        area += centerPointError * deltaTime;
        double output = kp * error + ki * area + kd * deltaError;
        lastError = error;
        lastTime = millis();
        return output;
    }
}

#endif
