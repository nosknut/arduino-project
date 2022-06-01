#include <ArduinoJson.h>
#include <Arduino.h>

class Timer {
private:
    unsigned long startTime;

public:
    Timer() {
        restart();
    }

    void restart() {
        startTime = millis();
    }
    
    unsigned long getElapsedTime() {
        return millis() - startTime;
    }

    bool isFinished(unsigned long duration) {
        return getElapsedTime() >= duration;
    }

    bool loopWait(unsigned long duration) {
        if (isFinished(duration)) {
            restart();
            return true;
        } else {
        return false;
        }
    }
};


DynamicJsonDocument jsonDoc = DynamicJsonDocument(200);

void setup()
{
    jsonDoc["topic"] = "encoders";
    jsonDoc["data_l"] = (int16_t)0;
    jsonDoc["data_r"] = (int16_t)0;
    jsonDoc.shrinkToFit();
}

void loop()
{
    if (timer.loopWait(10))
    {
        if ((rightTicks != prevRightTicks) || (leftTicks != prevLeftTicks))
        {
            jsonDoc["data_r"] = rightTicks;
            jsonDoc["data_l"] = leftTicks;
            serializeJson(jsonDoc, Serial);
        }
    }