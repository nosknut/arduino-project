#include <Arduino.h>
#include <ArduinoJson.h>

class Timer
{
private:
    unsigned long startTime;

public:
    Timer()
    {
        restart();
    }

    void restart()
    {
        startTime = millis();
    }

    unsigned long getElapsedTime()
    {
        return millis() - startTime;
    }

    bool isFinished(unsigned long duration)
    {
        return getElapsedTime() >= duration;
    }

    bool loopWait(unsigned long duration)
    {
        if (isFinished(duration))
        {
            restart();
            return true;
        }
        else
        {
            return false;
        }
    }
};

class TemperatureSensor
{
private:
    int temperature = 0;

public:
    int getTemperature()
    {
        temperature += 1;
        return temperature;
    }
};

class SoftwareBattery
{
private:
    int maxCapacity = 100;
    int capacity = maxCapacity;

public:
    void reset()
    {
        capacity = maxCapacity;
    }

    int getCapacity()
    {
        if (capacity > 0)
        {
            capacity -= 1;
        }
        else
        {
            capacity = maxCapacity;
        }

        return capacity;
    }
};

Timer timer;
TemperatureSensor temperatureSensor;
SoftwareBattery softwareBattery;

DynamicJsonDocument jsonDoc = DynamicJsonDocument(200);

void updateJsonDoc()
{
    jsonDoc["temperature"] = temperatureSensor.getTemperature();
    jsonDoc["battery"] = softwareBattery.getCapacity();
}

void setup()
{
    Serial.begin(9600);
    updateJsonDoc();
    jsonDoc.shrinkToFit();
}

void loop()
{
    if (timer.loopWait(500))
    {
        updateJsonDoc();
        serializeJson(jsonDoc, Serial);
        Serial.println("");
    }
}
