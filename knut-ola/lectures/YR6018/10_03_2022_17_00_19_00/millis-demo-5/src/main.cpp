#include <Arduino.h>

void setup()
{
    Serial.begin(9600);
}

void print(String message)
{
    Serial.println(message);
}

void loop()
{
}
