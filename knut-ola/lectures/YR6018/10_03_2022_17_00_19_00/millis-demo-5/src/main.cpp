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
long tidspunktKnappBleTryktNed = 0;
int tidKnappSkalHoldesNede = 3000;
bool viHarReagertPaAtKnappenBleHoldtNede = false;

bool hondterKnappetrykk()
{
    long detteTidspunktet = millis();
    bool knappErTryktNed = digitalRead(knappPin);

    if (knappErTryktNed && !knappVarTryktNedForigeGang)
    {
        tidspunktKnappBleTryktNed = detteTidspunktet;
        viHarReagertPaAtKnappenBleHoldtNede = false;
    }

    long tidSidenKnappenBleTryktNed = detteTidspunktet - tidspunktKnappBleTryktNed;
    bool knappenBleNedeLengeNokk = tidSidenKnappenBleTryktNed > tidKnappSkalHoldesNede;

    if (knappErTryktNed && knappenBleNedeLengeNokk)
    {
        if (!viHarReagertPaAtKnappenBleHoldtNede)
        {
            viHarReagertPaAtKnappenBleHoldtNede = true;
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
