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
    ros::Publisher right_pub_msg = ros::Publisher("right_ticks", &right_wheel_tick_count);

    std_msgs::Int16 left_wheel_tick_count;
    ros::Publisher left_pub_msg = ros::Publisher("left_ticks", &left_wheel_tick_count);

    Zumo32U4Encoders encoders;

public:
    void setup(ros::NodeHandle &nh)
    {
        nh.advertise(right_pub_msg);
        nh.advertise(left_pub_msg);
    }

    void loop(ros::NodeHandle &nh)
    {
        // publish the range value every 50 milliseconds
        //   since it takes that long for the sensor to stabilize
        if (timer.loopWait(100))
        {
            right_wheel_tick_count.data = encoders.getCountsLeft();
            left_wheel_tick_count.data = encoders.getCountsRight();

            right_pub_msg.publish(&right_wheel_tick_count);
            left_pub_msg.publish(&left_wheel_tick_count);
        }
    }
};

#endif
