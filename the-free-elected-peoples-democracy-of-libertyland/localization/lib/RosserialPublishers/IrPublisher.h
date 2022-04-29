#ifndef IrPublisher_h
#define IrPublisher_h
#include <Arduino.h>
#include <Timer.h>
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <Zumo32U4ProximitySensors.h>
#include <SerialClass.h>
#include <ArduinoJson.h>

/*
 * http://wiki.ros.org/rosserial_arduino/Tutorials/IR%20Ranger
 * rosserial IR Ranger Example
 *
 * This example is calibrated for the Sharp GP2D120XJ00F.
 */

class IrPublisher
{
private:
    Timer timer;
    Zumo32U4ProximitySensors proximity;

    DynamicJsonDocument leftOutDoc = DynamicJsonDocument(200);
    DynamicJsonDocument rightOutDoc = DynamicJsonDocument(200);
    DynamicJsonDocument frontLeftOutDoc = DynamicJsonDocument(200);
    DynamicJsonDocument frontRightOutDoc = DynamicJsonDocument(200);

public:
    void setup()
    {
        leftOutDoc["topic"] = "l_range";
        leftOutDoc["r"] = (uint8_t)0;
        leftOutDoc.shrinkToFit();

        rightOutDoc["topic"] = "r_range";
        rightOutDoc["r"] = (uint8_t)0;
        rightOutDoc.shrinkToFit();

        frontLeftOutDoc["topic"] = "fl_range";
        frontLeftOutDoc["r"] = (uint8_t)0;
        frontLeftOutDoc.shrinkToFit();

        frontRightOutDoc["topic"] = "fr_range";
        frontRightOutDoc["r"] = (uint8_t)0;
        frontRightOutDoc.shrinkToFit();

        proximity.initThreeSensors();
    }

    void printRangeFor(DynamicJsonDocument &outDoc, uint8_t counts)
    {
        uint8_t prevCounts = outDoc["r"];
        if (counts != prevCounts)
        {
            outDoc["r"] = counts;
            serializeJson(outDoc, DATA_SERIAL_CLASS);
        }
    }

    void loop()
    {
        proximity.read();
        // publish the range value every >= 50 milliseconds
        //   since it takes that long for the sensor to stabilize
        if (timer.loopWait(100))
        {
            printRangeFor(leftOutDoc, proximity.countsLeftWithLeftLeds());
            printRangeFor(rightOutDoc, proximity.countsRightWithRightLeds());
            printRangeFor(frontLeftOutDoc, proximity.countsFrontWithLeftLeds());
            printRangeFor(frontRightOutDoc, proximity.countsFrontWithRightLeds());
        }
    }
};

#endif
