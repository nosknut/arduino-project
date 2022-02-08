#include <ezButton.h>

const unsigned long eventInterval = 1000;
unsigned long prevInterval = 0;

int redLED = 10;    //RGB-rød
int greenLED = 9;   //RGB-grønn
int buzzerPin = 11; //Buzzer

int LED1 = 6;     //Spiller 1 - LED
int LED2 = 7;     //Spiller 2	- LED
ezButton SW1 = 3; //Spiller 1 - knapp
ezButton SW2 = 4; //Spiller 2 - knapp

int winner = 0;       //Initialiserer vinnerindikatoren
int winnerBeep = 750; //Buzzer-pitchen for vinnerfanfaren
int fault = 0;        //Initialiserer feilindikatoren
int faultBeep = 200;  //Buzzer-pitchen for feillyden

unsigned long wait = 0;
unsigned long now = 0;

void setup()
{
    Serial.begin(9600);
    pinMode(redLED, OUTPUT);
    pinMode(greenLED, OUTPUT);
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
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

void loop()
{
    SW1.loop();
    SW2.loop();

    const long endTime = millis() + 100;
    if (millis() >= endTime)
    {
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