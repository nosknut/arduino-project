#include <Arduino.h>

void setup()
{
    Serial.begin(9600);
}

void print(String message)
{
    Serial.println(message);
}

// A
// tidA er tidspunktet A kjorte sist
long tidspunktetAKjorteSist = 0;
// venteTidA er hvor lang tid det skal ta
// mellom hver gang A-koden skal kjore
int venteTidA = 1000;
bool tilstandA = false;

// B
long tidspunktetBKjorteSist = 0;
int venteTidB = 500;
bool tilstandB = false;

// C
// tidC er neste tidspunktet C skal kjore
long tidspunktetCskalKjore = 0;
int venteTidC = 3000;
bool tilstandC = false;

// D
long tidspunktetDskalKjore = 0;
int venteTidD = 4000;
bool tilstandD = false;

void loop()
{
    long detteTidspunktet = millis();

    ////////////////////////////////////////////
    ////Lagre tidspunkt sist gang noe kjorte////
    ////////////////////////////////////////////

    // A
    // Hvis forskjellen mellom dette tidspunktet og tidspunktet
    // hvor A kjorte sist er større enn ett sekund
    if ((detteTidspunktet - tidspunktetAKjorteSist) > venteTidA)
    {
        print("1 sekund har gatt og A er " + String(tilstandA));
        // Sett tilstandA til det motsatte av tilstandA
        // Også kjent som "Toggle" eller "Inverter"
        tilstandA = !tilstandA;
        // Lagre dette tidspunktet i tidspunktet A kjorte sist
        tidspunktetAKjorteSist = detteTidspunktet;
    }

    // B
    if ((detteTidspunktet - tidspunktetBKjorteSist) > venteTidB)
    {
        print("0.5 sekund har gatt og B er " + String(tilstandB));
        tilstandB = !tilstandB;
        tidspunktetBKjorteSist = detteTidspunktet;
    }

    //////////////////////////////////////////////////
    ////Lagre tidspunkt neste gang noe skal kjorte////
    //////////////////////////////////////////////////

    // C
    // Hvis dette tidspunktet er storre/senere enn tidspunktet C skal kjore
    if (detteTidspunktet > tidspunktetCskalKjore)
    {
        print("3 sekunder har gatt og C er " + String(tilstandC));
        tilstandC = !tilstandC;
        // 3 formuleringer av hva som skal
        // skje pa neste kodelinje:
        // C skal kjore om 3 sekunder
        // C skal kjore 3 sekunder fra dette tidspunktet
        // tidspunktet C skal kjore = dette tidspunktet + 3 sekunder
        tidspunktetCskalKjore = detteTidspunktet + venteTidC;
    }

    // D
    if (detteTidspunktet > tidspunktetDskalKjore)
    {
        print("4 sekunder har gatt og D er " + String(tilstandD));
        tilstandD = !tilstandD;
        tidspunktetDskalKjore = detteTidspunktet + venteTidD;
    }
}
