#include <Arduino.h>
#include <Zumo32U4.h>

Zumo32U4LCD lcd;

void blockingCountdown(const String message, const int durationMillis)
{
    for (int i = durationMillis; i > 0; i--)
    {
        lcd.clear();
        lcd.gotoXY(0, 0);
        lcd.print(message);
        lcd.gotoXY(0, 1);
        lcd.print(i);
        delay(1000);
    }
    lcd.clear();
}

void setup()
{
    Serial.begin(9600);
    blockingCountdown("Starting ...", 3);
}

void loop()
{
    delay(1);
}
