#ifndef MainPublisher_h
#define MainPublisher_h

// Uncomment this line to print data to the usb serial port
// #define DEBUG_ZUMO

#include <Arduino.h>
#include <IrPublisher.h>
#include <EncoderPublisher.h>
#include <ImuPublisher.h>

class MainPublisher
{
private:
    bool wasConnected = false;

    IrPublisher irPublisher;
    EncoderPublisher encoderPublisher;
    ImuPublisher imuPublisher;

public:
    void setup(long baudRate)
    {
        irPublisher.setup();
        encoderPublisher.setup();
        imuPublisher.setup();
    }

    void loop()
    {
        irPublisher.loop();
        encoderPublisher.loop();
        imuPublisher.loop();
    }
};

#endif
