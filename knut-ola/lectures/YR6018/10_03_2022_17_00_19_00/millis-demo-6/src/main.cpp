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

bool hondterKnappetrykk()
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
            viHarReagertPaAtKnappenBleHoldtNede = true;
            knappVarTryktNedForigeGang = knappErTryktNed;
            return true;
        }
    }

    knappVarTryktNedForigeGang = knappErTryktNed;
    return false;
}

int gangerTrykket = 0;

void loop()
{
    if (hondterKnappetrykk())
    {
        gangerTrykket++;
        switch (gangerTrykket)
        {
        case 1:
            print("Du har trykket 1 gang");
            break;
        case 2:
            print("Du har trykket 2 ganger");
            break;
        case 3:
            print("Du har trykket 3 ganger");
            print("Vi starter pa nytt");
            gangerTrykket = 0;
            break;
        default:
            gangerTrykket = 0;
            break;
        }
    }
}
