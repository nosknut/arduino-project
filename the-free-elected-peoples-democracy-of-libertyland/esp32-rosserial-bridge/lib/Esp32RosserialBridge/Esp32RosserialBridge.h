#ifndef Esp32RosserialBridge_h
#define Esp32RosserialBridge_h
#include <Arduino.h>
#include <MainRosserialBridge.h>
#include <ESP8266WiFi.h>

class Esp32RosserialBridge
{
private:
    MainRosserialBridge mainRosserialBridge;

    void setupWifi(String ssid, String password)
    {
        Serial.println();
        Serial.print("Connecting to ");
        Serial.println(ssid);

        // Connect the ESP8266 the the wifi AP
        WiFi.begin(ssid, password);
        while (WiFi.status() != WL_CONNECTED)
        {
            delay(500);
            Serial.print(".");
        }
        Serial.println("");
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
    }

public:
    void setup(long baudRate, String ssid, String password, uint16_t rosserialServerPort = 11411)
    {
        setupWifi(ssid, password);
        mainRosserialBridge.setup(baudRate, rosserialServerPort);
    }

    void loop()
    {
        mainRosserialBridge.loop();
    }
};

#endif
