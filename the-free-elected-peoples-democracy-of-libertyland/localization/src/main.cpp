#include <Arduino.h>
#include <MainPublisher.h>
#include <MainSubscriber.h>

long baudRate = 115200;
MainPublisher mainPublisher;
MainSubscriber mainSubscriber;

void setup()
{
    Serial.begin(baudRate);
    Serial1.begin(baudRate);
    mainPublisher.setup();
}

void loop()
{
    mainPublisher.loop();
    mainSubscriber.loop();
}
