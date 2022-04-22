#include <Arduino.h>
#include <MainPublisher.h>

long baudRate = 115200;
MainPublisher mainPublisher;

void setup()
{
    mainPublisher.setup(baudRate);
}

void loop()
{
    mainPublisher.loop();
}
