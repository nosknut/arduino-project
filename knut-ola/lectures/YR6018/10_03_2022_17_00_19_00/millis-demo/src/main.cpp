#include <Arduino.h>

void setup()
{
    Serial.begin(9600);
}

void print(String message)
{
    Serial.println(message);
}

bool ledErPa = false;

int blinklysDelay = 500;
long blinkLysTid = millis();

int manglerSetebelteAlarmDelay = 3000;
long setebelteTid = millis();
void loop()
{
    long nu = millis();
    if ((nu - blinkLysTid) > blinklysDelay)
    {
        blinkLysTid = nu;
        ledErPa = !ledErPa;
        digitalWrite(LED_BUILTIN, ledErPa);
    }

    if ((nu - setebelteTid) > manglerSetebelteAlarmDelay)
    {
        setebelteTid = nu;
        print("Mangler setebelte alarm");
    }
}
