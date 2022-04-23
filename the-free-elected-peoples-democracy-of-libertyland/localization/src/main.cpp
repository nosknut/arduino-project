#include <Arduino.h>
#include <MainPublisher.h>

long baudRate = 115200;
MainPublisher mainPublisher;

void setup()
{
    Serial.begin(baudRate);
    mainPublisher.setup(baudRate);
}

void loop()
{
    mainPublisher.loop();
}
