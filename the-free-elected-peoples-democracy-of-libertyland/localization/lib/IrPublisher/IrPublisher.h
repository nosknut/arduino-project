#ifndef IrPublisher_h
#define IrPublisher_h
#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <Zumo32U4ProximitySensors.h>
#include <ros.h>;
#include <ros/time.h>;
#include <sensor_msgs/Range.h>;

/*
 * rosserial IR Ranger Example
 *
 * This example is calibrated for the Sharp GP2D120XJ00F.
 */
class IrPublisher
{
private:
    ros::NodeHandle nh;
    sensor_msgs::Range range_msg;
    ros::Publisher pub_range("range_data", &range_msg);

    unsigned long range_timer;

    Zumo32U4ProximitySensors proximity;

public:
    IrPublisher()
    {
    }

    char frameid[] = "/ir_ranger";

    void setup()
    {
        nh.initNode();
        nh.advertise(pub_range);

        range_msg.radiation_type = sensor_msgs::Range::INFRARED;
        range_msg.header.frame_id = frameid;
        range_msg.field_of_view = 0.01;
        range_msg.min_range = 0.03; // For GP2D120XJ00F only. Adjust for other IR rangers
        range_msg.max_range = 0.4;  // For GP2D120XJ00F only. Adjust for other IR rangers
    }

    void loop()
    {
        proximity.read();
        // publish the range value every 50 milliseconds
        //   since it takes that long for the sensor to stabilize
        if ((millis() - range_timer) > 50)
        {
            range_msg.range = proximity.countsFrontWithRightLeds();
            range_msg.header.stamp = nh.now();
            pub_range.publish(&range_msg);
            range_timer = millis();
        }
        nh.spinOnce();
    }
};

#endif
