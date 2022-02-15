#include <ezButton.h>

int redLED = 10;    // RGB-rød
int greenLED = 9;   // RGB-grønn
int buzzerPin = 11; // Buzzer

int LED1 = 6;     // Spiller 1 - LED
int LED2 = 7;     // Spiller 2	- LED
ezButton SW1 = 3; // Spiller 1 - knapp
ezButton SW2 = 4; // Spiller 2 - knapp

int winner = 0;       // Initialiserer vinnerindikatoren
int winnerBeep = 750; // Buzzer-pitchen for vinnerfanfaren
int fault = 0;        // Initialiserer feilindikatoren
int faultBeep = 200;  // Buzzer-pitchen for feillyden

unsigned long wait = 0;
unsigned long now = 0;
// random variabler
long randomTall;
const int seedPin = A0;

void setup()
{
    Serial.begin(9600);
    pinMode(redLED, OUTPUT);
    pinMode(greenLED, OUTPUT);
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);

    // random seed og tall
    randomSeed(analogRead(seedPin));
    randomTall = random(3, 7);
}
void buttonStart()
{
    SW1.loop();
    SW2.loop();

    SW1.setDebounceTime(50);
    SW2.setDebounceTime(50);
}
// mulighet for winner og taper loop etter ferdig spill:
/* variabler:
bool gameOver = false;
bool gameWon = false;
// true = player 1, false = player 2
bool vinner;*/
// funk sjekkVinner:
/*void sjekkVinner(int LED)
{
    // sjekker hvem som vinner
    if (LED = 6)
    {
        vinner = true;
    }
    if (LED = 7)
    {
        vinner = false;
    }
}*/
// legg til:
/*gameWon = true;
sjekkVinner(ledPin);*/
// i vinner og taper funksjoner for å komme inn i while loops

/*while (gameOver)
{
    buttonStart();
    // Serial.flush();
    Serial.print("Game Over....");
    /*if (vinner)
    {
        Serial.println("Taperen er spiller 1, du er for rask, trykk når den er grønn");
    }
    if (vinner = false)
    {
        Serial.println("Taperen er spiller 2, du er for rask, trykk når den er grønn");
    }*/

// vinnerFanfaren funksjon, tar inn LED-pinne og bruker millis()
/*void vinnerFanfarenMedMillis(int ledPin)
{
    bool loop = true;
    int i = 1;
    while (loop)
    {
        int buzzerTone = 10000 - 750 * i;
        tone(buzzerPin, buzzerTone);
        if (millis() % 200 == 0)
        {
            digitalWrite(redLED, HIGH);
            digitalWrite(ledPin, LOW);
            i++;
        }
        else
        {
            digitalWrite(redLED, LOW);
            digitalWrite(ledPin, HIGH);
        }
        if (i * 750 > 10000)
        {
            loop = false;
        }
    }
    digitalWrite(redLED, LOW);
    digitalWrite(ledPin, LOW);
    noTone(buzzerPin);
}

Serial.println("Press Any Button To Restart");

if (SW1.isPressed())
{
    gameOver = false;
}
if (SW2.isPressed())
{
    gameOver = false;
}
}
while (gameWon)
{
    buttonStart();
    Serial.print("Game Won!!!");
    if (vinner)
    {
        Serial.println("Vinneren er spiller 1! Gratulerer");
    }
    if (vinner = false)
    {
        Serial.println("Vinneren er spiller 2! Gratulerer");
    }

    Serial.println("Press Any Button To Restart");
    if (SW1.isPressed())
    {
        gameWon = false;
    }
    if (SW2.isPressed())
    {
        gameWon = false;
    }
}*/

void loop()
{
    buttonStart();
    // Serial.println(randomTall);

    const long time = millis();
    const long timeRedLed = (randomTall * 1000);
    const long timeGreenLed = timeRedLed + 1000;

    const long diff = timeRedLed - time;
    Serial.println(diff);

    if (time < timeRedLed)
    {
        // digitalWrite(redLED, HIGH);
        // digitalWrite(greenLED, LOW);
        Serial.println("RedLedLyser");
    }
    if (time >= timeRedLed && time <= timeGreenLed)
    {
        // digitalWrite(redLED, LOW);
        // digitalWrite(greenLED, HIGH);
        Serial.print("GreenLedLyser");
    }
    if (time > timeGreenLed)
    {
        Serial.println("Begge er av");
    }
}