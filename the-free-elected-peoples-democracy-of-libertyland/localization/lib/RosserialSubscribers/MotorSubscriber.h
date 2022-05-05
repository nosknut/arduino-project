#ifndef MotorSubscriber_h
#define MotorSubscriber_h
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Timer.h>
#include <SerialClass.h>
#include <Zumo32U4Motors.h>
#include <PidController.h>
#include <PidControllerConfig.h>
#include <Range.h>
#include <RunningMedian.h>

Zumo32U4Motors motors;
Zumo32U4Encoders encoders;

class WheelController
{
private:
    Timer timer;
    int16_t minSpeed = 50;
    int16_t prevTicks = 0;
    int ticksPerMeter = 8000;
    float targetTps = 0;
    RunningMedian filter = RunningMedian(1);
    PidControllerConfig config = PidControllerConfig(0.001, 0.002, 0, Range(-1, 1), Range(-400, 400));
    PidController pidController = PidController(config);

public:
    void setTargetSpeed(float newTargetSpeed)
    {
        targetTps = newTargetSpeed * ticksPerMeter;
    }

    virtual int16_t getTicks() = 0;
    virtual void setSpeed(int16_t speed) = 0;

    void update(bool debug)
    {
        if(targetTps == 0)
        {
            setSpeed(0);
            return;
        }
        if (timer.isFinished(1))
        {
            unsigned long duration = timer.getElapsedTimeMicros();
            timer.reset();
            int16_t currentTicks = getTicks();
            int16_t tps = (currentTicks - prevTicks) * (1000000.0 / duration);
            filter.add(tps);
            prevTicks = currentTicks;

            int ticksPerSec = filter.getMedian();

            // Handle wrap around
            if (abs(ticksPerSec) > 50000)
            {
                Serial.println("Encoder looped around");
                return;
            }

            int cmd = -pidController.update(ticksPerSec, targetTps, true);

            if (debug)
            {
                Serial.print(targetTps);
                Serial.print(" ");
                Serial.print(ticksPerSec);
                Serial.print(" ");
                Serial.print(tps);
                Serial.print(" ");
                Serial.println(cmd);
            }

            if (abs(cmd) > minSpeed)
            {
                setSpeed(constrain(cmd, -400, 400));
            }
            else
            {
                setSpeed(0);
            }
        }
    }
};

class RightWheelController : public WheelController
{
private:
    void setSpeed(int16_t speed)
    {
        motors.setRightSpeed(speed);
    }
    int16_t getTicks()
    {
        return encoders.getCountsRight();
    }
};

class LeftWheelController : public WheelController
{
private:
    void setSpeed(int16_t speed)
    {
        motors.setLeftSpeed(speed);
    }
    int16_t getTicks()
    {
        return encoders.getCountsLeft();
    }
};

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
    LeftWheelController leftWheelController;
    RightWheelController rightWheelController;

public:
    void loop(JsonDocument &inputDoc)
    {
        if (inputDoc["topic"] == "rmotor_cmd")
        {
            timeSinceReceived.reset();
            float right = inputDoc["data"];
            rightWheelController.setTargetSpeed(right);
        }

        if (inputDoc["topic"] == "lmotor_cmd")
        {
            timeSinceReceived.reset();
            float left = inputDoc["data"];
            leftWheelController.setTargetSpeed(left);
        }
    }

    void securityLoop()
    {
        // Kill the motors if we haven't received a message in a while
        if (timeSinceReceived.loopWait(motorCommandTimeout))
        {
            leftWheelController.setTargetSpeed(0);
            rightWheelController.setTargetSpeed(0);
        }
        leftWheelController.update(false);
        rightWheelController.update(false);
    }
};

#endif
