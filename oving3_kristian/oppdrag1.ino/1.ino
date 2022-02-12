#include <ezButton.h>

const unsigned long eventInterval = 1000;
unsigned long prevInterval = 0;

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

void vinnerFanfaren(int ledPin)
{
    for (int i = 1; i < 10; i++)
    {
        tone(buzzerPin, 750 * i);
        if (i % 2 == 0)
        {
            digitalWrite(greenLED, HIGH);
            digitalWrite(ledPin, LOW);
        }
        else
        {
            digitalWrite(greenLED, LOW);
            digitalWrite(ledPin, HIGH);
        }
        delay(100);
    }
    digitalWrite(greenLED, LOW);
    digitalWrite(ledPin, LOW);
    delay(500);
    noTone(buzzerPin);
}

void feilLyd(int ledPin)
{
    for (int i = 10; i > 1; i--)
    {
        tone(buzzerPin, 750 * i);
        if (i % 2 == 0)
        {
            digitalWrite(redLED, HIGH);
            digitalWrite(ledPin, LOW);
        }
        else
        {
            digitalWrite(redLED, LOW);
            digitalWrite(ledPin, HIGH);
        }
        delay(100);
    }
    digitalWrite(redLED, LOW);
    digitalWrite(ledPin, LOW);
    delay(500);
    noTone(buzzerPin);
}

void buttonStart()
{
    SW1.loop();
    SW2.loop();

    SW1.setDebounceTime(50);
    SW2.setDebounceTime(50);
}

void loop()
{
    buttonStart();

    const long timeRedLed = millis() + (randomTall * 1000);
    if (millis() < timeRedLed)
    {
        digitalWrite(redLED, HIGH);
        Serial.println("RedLedLyser");
    }
    if (millis() >= (randomTall * 1000))
    {
        digitalWrite(redLED, LOW);
        digitalWrite(greenLED, HIGH);
    }

    if (SW1.isPressed())
    {
        vinnerFanfaren(LED1);
    }
    if (SW2.isPressed())
    {
        feilLyd(LED2);
    }
}