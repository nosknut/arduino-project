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

    DynamicJsonDocument outputDocument = DynamicJsonDocument(30);

public:
    void setup()
    {
        outputDocument["topic"] = "range_data";
        outputDocument["header"]["frame_id"] = "/ir_ranger";
        outputDocument["radiation_type"] = sensor_msgs::Range::INFRARED;
        outputDocument["field_of_view"] = 0.01;
        outputDocument["min_range"] = 0.03; // For GP2D120XJ00F only. Adjust for other IR rangers
        outputDocument["max_range"] = 0.4;  // For GP2D120XJ00F only. Adjust for other IR rangers
    }

    void loop()
    {
        proximity.read();
        // publish the range value every 50 milliseconds
        //   since it takes that long for the sensor to stabilize
        if (timer.loopWait(100))
        {
            auto currentRange = proximity.countsFrontWithRightLeds();
            if (currentRange != outputDocument["range"])
            {
                outputDocument["range"] = currentRange;
                serializeJson(outputDocument, SERIAL_CLASS);
            }
        }
    }
};

#endif
