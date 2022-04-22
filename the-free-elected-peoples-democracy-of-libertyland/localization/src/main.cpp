#include <Arduino.h>
#include <MainPublisher.h>

MainPublisher mainPublisher;

void setup()
{
    mainPublisher.setup();
}

void loop()
{
    mainPublisher.loop();
}
