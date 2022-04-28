#include <Arduino.h>
#include <WiFi.h>
#include <Esp32RosserialBridge.h>

long baudRate = 115200;
String ssid = "";
String password = "";
uint16_t rosserialServerPort = 11411;
// IP address to the rosserial socket server
IPAddress serverIp = IPAddress(192, 168, 1, 190);

Esp32RosserialBridge bridge;

void setup()
{
    Serial.begin(baudRate);
    bridge.setup(serverIp, baudRate, ssid, password, rosserialServerPort);
}

void loop()
{
    bridge.loop();
}
