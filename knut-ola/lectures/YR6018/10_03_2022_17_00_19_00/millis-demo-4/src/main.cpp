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

void loop()
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
            print("Knapp ble holdt nede i 3 sekunder");
            viHarReagertPaAtKnappenBleHoldtNede = true;
        }
    }

    knappVarTryktNedForigeGang = knappErTryktNed;
}
