long randomTall;
//ønsket å bruke array og for-løkker for å gjøre
//programmet fint, men har ikke lært alt for å få det til
//og fant ikke noe på nettet som ga noe mening, så det blir hardkoding:
int tall1 = 0;
int tall2 = 0;
int tall3 = 0;
int tall4 = 0;
int tall5 = 0;
int tall6 = 0;
// konstant for knapp og lys
const int redLed = 9;
const int knapp = 7;
// tilstands variabel
bool tilstand;

void setup()
{
    pinMode(redLed, OUTPUT);
    pinMode(knapp, INPUT);
    Serial.begin(9600);
    randomSeed(analogRead(A0));
}

// Printer input tall, hva det er (x) og hvor mange prosent mot 10 000
void printTallStat(int inputTall, int x)
{
    //------Tall x Print---------
    Serial.println(" ");
    Serial.print("Tallet ");
    Serial.print(x);
    Serial.print(" ble generert ");
    Serial.print(inputTall);
    Serial.print(" ganger og det tilsvarer ");
    Serial.print(inputTall / 100);
    Serial.print(" prosent.");
}

// putter inn tall som skal testes og tall det testes mot og ser om de er like
int testTall(int inputTall, int x)
{
    int i = 0;
    if (inputTall == x)
    {
        i++;
    }
    return i;
}

// Legger alt sammen
void testTallOgPrint()
{
    for (int i = 0; i < 10001; i++)
    {
        // Lager random tall med radomSeed()
        randomTall = random(1, 7);
        // Tester alle tall
        tall1 += testTall(randomTall, 1);
        tall2 += testTall(randomTall, 2);
        tall3 += testTall(randomTall, 3);
        tall4 += testTall(randomTall, 4);
        tall5 += testTall(randomTall, 5);
        tall6 += testTall(randomTall, 6);
    }
    // Printer alle tall og stat
    printTallStat(tall1, 1);
    printTallStat(tall2, 2);
    printTallStat(tall3, 3);
    printTallStat(tall4, 4);
    printTallStat(tall5, 5);
    printTallStat(tall6, 6);
    Serial.println(" ");
    Serial.print("____________Ny data____________");
    // Setter alle tall til 0, så vi ikke får feil verdier
    tall1 = 0;
    tall2 = 0;
    tall3 = 0;
    tall4 = 0;
    tall5 = 0;
    tall6 = 0;
    delay(1000);
}

void loop()
{
    tilstand = digitalRead(knapp);
    //Serial.println(tilstand);
    if (tilstand == true)
    {
        digitalWrite(redLed, HIGH);
        testTallOgPrint();
    }
    else
    {
        digitalWrite(redLed, LOW);
    }
    delay(100);
}