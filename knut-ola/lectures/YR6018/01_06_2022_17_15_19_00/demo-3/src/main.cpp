#include <Arduino.h>
#include <ArduinoJson.h>

DynamicJsonDocument jsonDoc = DynamicJsonDocument(200);

void setup()
{
    Serial.begin(9600);
}

bool readData()
{
    if (Serial.available())
    {
        DeserializationError err = deserializeJson(jsonDoc, Serial);

        if (err == DeserializationError::Ok)
        {
            return true;
        }
    }
    return false;
}

void loop()
{
    if (readData())
    {
        int temperature = jsonDoc["temperature"];
        int battery = jsonDoc["battery"];

        Serial.print("Temperature: ");
        Serial.println(temperature);

        Serial.print("Battery: ");
        Serial.println(battery);
    }
}
