#ifndef MainSubscriber_h
#define MainSubscriber_h
#include <Arduino.h>
#include <ArduinoJson.h>
#include <SerialClass.h>
#include <MotorSubscriber.h>

class MainSubscriber
{
private:
    MotorSubscriber motorSubscriber;

    DynamicJsonDocument inputDoc = DynamicJsonDocument(80);
    SerialClass &inputStream = DATA_SERIAL_CLASS;

public:
    void loop()
    {

        if (inputStream.available())
        {
            // Read the JSON document from the "link" serial port
            DeserializationError err = deserializeJson(inputDoc, inputStream);

            if (err == DeserializationError::Ok)
            {
                motorSubscriber.loop(inputDoc);
            }
            // else
            // {
            //     Serial.println("Deserialization error: " + String(err.c_str()));
            //     inputStream.read();
            // }
        }
        motorSubscriber.securityLoop();
    }
};

#endif
