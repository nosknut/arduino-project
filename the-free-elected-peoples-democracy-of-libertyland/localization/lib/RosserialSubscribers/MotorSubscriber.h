#ifndef MotorSubscriber_h
#define MotorSubscriber_h
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Timer.h>
#include <SerialClass.h>
#include <Zumo32U4Motors.h>

/*
 * https://automaticaddison.com/how-to-publish-wheel-encoder-tick-data-using-ros-and-arduino/
 */
class MotorSubscriber
{
private:
    // Max amount of time between received messages
    // before motors stop (security measure)
    int motorCommandTimeout = 1000;
    Timer timeSinceReceived;
    Zumo32U4Motors motors;

public:
    void loop(JsonDocument &inputDoc)
    {
        if (inputDoc["topic"] == "rmotor_cmd")
        {
            timeSinceReceived.reset();
            int16_t left = inputDoc["data"];
            motors.setRightSpeed(left);
        }

        if (inputDoc["topic"] == "lmotor_cmd")
        {
            timeSinceReceived.reset();
            int16_t right = inputDoc["data"];
            motors.setLeftSpeed(right);
        }
    }

    void securityLoop()
    {
        // Kill the motors if we haven't received a message in a while
        if (timeSinceReceived.loopWait(motorCommandTimeout))
        {
            motors.setSpeeds(0, 0);
        }
    }
};

#endif
