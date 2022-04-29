#ifndef EncoderPublisher_h
#define EncoderPublisher_h
#include <Arduino.h>
#include <Timer.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>
#include <Zumo32U4Encoders.h>

/*
 * https://automaticaddison.com/how-to-publish-wheel-encoder-tick-data-using-ros-and-arduino/
 */
class EncoderPublisher
{
private:
    Timer timer;
    DynamicJsonDocument leftOutputDocument = DynamicJsonDocument(200);
    DynamicJsonDocument rightOutputDocument = DynamicJsonDocument(200);

    Zumo32U4Encoders encoders;

public:
    EncoderPublisher()
    {
        rightOutputDocument["topic"] = "right_ticks";
        rightOutputDocument["data"] = (int16_t)0;
        rightOutputDocument.shrinkToFit();

        leftOutputDocument["topic"] = "left_ticks";
        leftOutputDocument["data"] = (int16_t)0;
        leftOutputDocument.shrinkToFit();
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
            auto prevRightTicks = rightOutputDocument["data"];
            auto prevLeftTicks = leftOutputDocument["data"];
            if (rightTicks != prevRightTicks)
            {
                rightOutputDocument["data"] = rightTicks;
                serializeJson(rightOutputDocument, SERIAL_CLASS);
            }

            if (leftTicks != prevLeftTicks)
            {
                leftOutputDocument["data"] = leftTicks;
                serializeJson(leftOutputDocument, SERIAL_CLASS);
            }
        }
    }
};

#endif
