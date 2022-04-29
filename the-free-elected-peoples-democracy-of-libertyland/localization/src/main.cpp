#include <Arduino.h>
#include <MainPublisher.h>

long baudRate = 115200;
SerialConnection nh = SerialConnection(&SERIAL_CLASS);
// MainPublisher mainPublisher;

void setup()
{
    Serial.begin(baudRate);
    // mainPublisher.setup(baudRate);
}

struct Info
{
    String name;
    int value;
};

Info info;

void loop()
{
    if (SERIAL_CLASS.available())
    {
        Serial.print(SERIAL_CLASS.read());
    }
    // mainPublisher.loop();
    if (nh.readMessage("/info", info))
    {
        Serial.println(info.name);
        Serial.println(info.value);
    }
    nh.spinOnce();
}
