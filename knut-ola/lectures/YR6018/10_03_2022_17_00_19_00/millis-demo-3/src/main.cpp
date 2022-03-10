#include <Arduino.h>

void setup()
{
    Serial.begin(9600);
}

void print(String message)
{
    Serial.println(message);
}

long startTid = millis();
void loop()
{
    if ((millis() - startTid) > 1000)
    {
        print("1 sekund har gatt");
        startTid = millis();
    }
}
