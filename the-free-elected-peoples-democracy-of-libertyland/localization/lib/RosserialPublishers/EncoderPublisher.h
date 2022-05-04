#ifndef EncoderPublisher_h
#define EncoderPublisher_h
#include <Arduino.h>
#include <Timer.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>
#include <Zumo32U4Encoders.h>
#include <SerialClass.h>

/*
 * https://automaticaddison.com/how-to-publish-wheel-encoder-tick-data-using-ros-and-arduino/
 */
class EncoderPublisher
{
private:
    Timer timer;
    DynamicJsonDocument outputDocument = DynamicJsonDocument(200);

    Zumo32U4Encoders encoders;

public:
    EncoderPublisher()
    {
        outputDocument["topic"] = "encoders";
        outputDocument["data_l"] = (int16_t)0;
        outputDocument["data_r"] = (int16_t)0;
        outputDocument.shrinkToFit();
    }

    void setup()
    {
    }

    void loop()
    {
        if (timer.loopWait(10))
        {
            int16_t rightTicks = encoders.getCountsRight();
            int16_t leftTicks = encoders.getCountsLeft();
            auto prevRightTicks = outputDocument["data_r"];
            auto prevLeftTicks = outputDocument["data_l"];
            if ((rightTicks != prevRightTicks) || (leftTicks != prevLeftTicks))
            {
                outputDocument["data_r"] = rightTicks;
                outputDocument["data_l"] = leftTicks;
                serializeJson(outputDocument, DATA_SERIAL_CLASS);
            }
        }
    }
};

#endif
