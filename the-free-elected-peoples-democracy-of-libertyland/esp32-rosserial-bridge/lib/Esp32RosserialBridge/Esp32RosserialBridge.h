#ifndef Esp32RosserialBridge_h
#define Esp32RosserialBridge_h
#include <Arduino.h>
#include <MainRosserialBridge.h>
#include <WiFi.h>

class Esp32RosserialBridge
{
private:
    WiFi_NodeHandle outputNh;
    MainRosserialBridge mainRosserialBridge = MainRosserialBridge(outputNh);

    void setupWifi(String ssid, String password)
    {
        Serial.println();
        Serial.print("Connecting to ");
        Serial.println(ssid);
        outputNh.loginfo(("Connecting to " + ssid).c_str());
        // Connect the ESP8266 the the wifi AP
        WiFi.begin(ssid.c_str(), password.c_str());
        while (WiFi.status() != WL_CONNECTED)
        {
            delay(500);
            Serial.print(".");
        }
        Serial.println("");
        Serial.println("WiFi connected");
        outputNh.loginfo("Wifi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        outputNh.loginfo(("IP address: " + WiFi.localIP().toString()).c_str());
    }

public:
    void setup(IPAddress serverIpAddress, long baudRate, String ssid, String password, uint16_t rosserialServerPort = 11411)
    {
        setupWifi(ssid, password);
        mainRosserialBridge.setup(serverIpAddress, baudRate, rosserialServerPort);
    }

    void loop()
    {
        mainRosserialBridge.loop();
    }
};

#endif
