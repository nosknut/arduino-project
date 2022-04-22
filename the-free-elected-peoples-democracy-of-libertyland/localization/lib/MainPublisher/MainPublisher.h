#ifndef MainPublisher_h
#define MainPublisher_h
#include <Arduino.h>
#include <IrPublisher.h>
#include <EncoderPublisher.h>
#include <ImuPublisher.h>

class MainPublisher
{
private:
    ros::NodeHandle nh;

    IrPublisher irPublisher;
    EncoderPublisher encoderPublisher;
    ImuPublisher imuPublisher;

public:
    void setup(long baudRate)
    {
        nh.getHardware()->setBaud(baudRate);
        nh.initNode();

        irPublisher.setup(nh);
        encoderPublisher.setup(nh);
        imuPublisher.setup(nh);
    }

    void loop()
    {
        irPublisher.loop(nh);
        encoderPublisher.loop(nh);
        imuPublisher.loop(nh);

        nh.spinOnce();
    }
};

#endif
