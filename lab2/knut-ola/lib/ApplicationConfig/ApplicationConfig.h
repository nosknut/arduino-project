#ifndef ApplicationConfig_h
#define ApplicationConfig_h

#include <ServoConfig.h>
#include <MotorConfig.h>

struct ApplicationConfig
{
    const ServoConfig servoConfig = {
        .servoPin = 9,
        .angleLimits = Range(0, 180),
    };
    const MotorConfig motorConfig = {
        .enablePin = 11,
        .in1Pin = 12,
        .in2Pin = 13,
        // The motor is incapable of starting at speeds lower than 140.
        .speedLimits = Range(140, 255),
    };
    const int potmeterPin = A0;
    const int photoresistorPin = A1;
    const int emergencyStopPin = 2;
    const int baudRate = 9600;
} const appConfig;

#endif
