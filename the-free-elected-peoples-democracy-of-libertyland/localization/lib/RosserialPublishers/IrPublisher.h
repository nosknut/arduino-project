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

    DynamicJsonDocument outputDocument = DynamicJsonDocument(200);

public:
    void setup()
    {
        outputDocument["topic"] = "range_data";
        outputDocument["range"] = (uint8_t)0;

        outputDocument.shrinkToFit();
    }

    void loop()
    {
        proximity.read();
        // publish the range value every >= 50 milliseconds
        //   since it takes that long for the sensor to stabilize
        if (timer.loopWait(100))
        {
            uint8_t currentRange = proximity.countsFrontWithRightLeds();
            uint8_t prevRange = outputDocument["range"];
            if (abs(currentRange - prevRange) > 10)
            {
                outputDocument["range"] = currentRange;
                serializeJson(outputDocument, SERIAL_CLASS);
            }
        }
    }
};

#endif
