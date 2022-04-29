#include <Arduino.h>
#include <WiFi.h>
#include <Esp32RosserialBridge.h>

long baudRate = 115200;
String ssid = "Telenor1959bak";
String password = "yzefbjbqzgxqu";
uint16_t rosserialServerPort = 11411;
// IP address to the rosserial socket server
IPAddress serverIp = IPAddress(192, 168, 1, 190);
SerialConnection nh = SerialConnection(&Serial);

// Esp32RosserialBridge bridge;

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
    info.name = "test";
    info.value++;
    nh.publish("/info", &info);
    delay(1000);
    nh.spinOnce();
}
