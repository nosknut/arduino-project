#ifndef MainPublisher_h
#define MainPublisher_h
#include <Arduino.h>
#include <Timer.h>
#include <IrPublisher.h>
#include <EncoderPublisher.h>

class MainPublisher
{
private:
    Timer timer;
    ros::NodeHandle nh;
    IrPublisher irPublisher;
    EncoderPublisher encoderPublisher;

public:
    void setup()
    {
        nh.getHardware()->setBaud(115200);
        nh.initNode();
        irPublisher.setup(nh);
        encoderPublisher.setup(nh);
    }

    void loop(ros::NodeHandle &nh)
    {
        irPublisher.loop(nh);
        encoderPublisher.loop(nh);
        nh.spinOnce();
    }
};

#endif
