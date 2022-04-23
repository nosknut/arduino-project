#include <Arduino.h>
#include <Esp32RosserialBridge.h>

long baudRate = 115200;
String ssid = "";
String password = "";
uint16_t rosserialServerPort = 11411;

Esp32RosserialBridge bridge;

void setup()
{
    Serial.begin(baudRate);
    bridge.setup(baudRate, ssid, password, rosserialServerPort);
}

void loop()
{
    bridge.loop();
}
