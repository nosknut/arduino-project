#include <Arduino.h>

int knappPin = 2;

void setup()
{
    Serial.begin(9600);
    pinMode(knappPin, INPUT);
}

void print(String message)
{
    Serial.println(message);
}

bool knappVarTryktNedForigeGang = false;
bool viHarReagertPaAtKnappenBleHoldtNede = false;

void loop()
{
    bool knappErTryktNed = digitalRead(knappPin);

    if (knappErTryktNed && !knappVarTryktNedForigeGang)
    {
        viHarReagertPaAtKnappenBleHoldtNede = false;
    }

    if (knappErTryktNed)
    {
        if (!viHarReagertPaAtKnappenBleHoldtNede)
        {
            print("Knapp ble holdt nede i 3 sekunder");
            viHarReagertPaAtKnappenBleHoldtNede = true;
        }
    }

    knappVarTryktNedForigeGang = knappErTryktNed;
}
