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
    std_msgs::Int16 right_wheel_tick_count;
    // ros::Publisher right_pub_msg = ros::Publisher("right_ticks", &right_wheel_tick_count);

    std_msgs::Int16 left_wheel_tick_count;
    // ros::Publisher left_pub_msg = ros::Publisher("left_ticks", &left_wheel_tick_count);

    Zumo32U4Encoders encoders;

public:
    void setup(SerialConnection &nh)
    {
        // nh.advertise(right_pub_msg);
        // nh.advertise(left_pub_msg);
    }

    void loop(SerialConnection &nh)
    {
        // publish the range value every 50 milliseconds
        //   since it takes that long for the sensor to stabilize
        if (timer.loopWait(10))
        {
            int16_t right_ticks = encoders.getCountsRight();
            int16_t left_ticks = encoders.getCountsLeft();
            auto old_right_ticks = right_wheel_tick_count.data;
            auto old_left_ticks = left_wheel_tick_count.data;
            if (right_ticks != old_right_ticks)
            {
                right_wheel_tick_count.data = right_ticks;
                nh.publish("right_ticks", &right_wheel_tick_count);
            }

            if (left_ticks != old_left_ticks)
            {
                left_wheel_tick_count.data = left_ticks;
                nh.publish("left_ticks", &left_wheel_tick_count);
            }
        }
    }
};

#endif
