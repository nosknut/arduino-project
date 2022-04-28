#ifndef MainPublisher_h
#define MainPublisher_h
#include <Arduino.h>
#include <IrPublisher.h>
#include <EncoderPublisher.h>
#include <ImuPublisher.h>

class MainPublisher
{
private:
    bool wasConnected = false;

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

        if (nh.connected())
        {
            // Run publishers
            // The publishers for this class
            // are bridges created in setup()
            if (!wasConnected)
            {
                Serial.println("Connected!");
                wasConnected = true;
            }
        }
        else
        {
            if (wasConnected)
            {
                Serial.println("Disconnected");
                wasConnected = false;
            }
            else
            {
                Serial.println("Not Connected");
                delay(1000);
            }
        }

        nh.spinOnce();
    }
};

#endif
