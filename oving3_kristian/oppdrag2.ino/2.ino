#include <ezButton.h>

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

void loop()
{
    SW1.loop();
    SW2.loop();

    if (SW1.isPressed())
    {
        Serial.println("Spiller 1 trykk");
        digitalWrite(LED1, HIGH);
        digitalWrite(greenLED, HIGH);
    }

    if (SW2.isPressed())
    {
        Serial.println("Spiller 2 trykk");
        digitalWrite(LED2, HIGH);
        digitalWrite(redLED, HIGH);
    }
    // if release
    if (SW1.isReleased())
    {
        Serial.println("Spiller 1 av");
        digitalWrite(LED1, LOW);
        digitalWrite(greenLED, LOW);
    }

    if (SW2.isReleased())
    {
        Serial.println("Spiller 2 av");
        digitalWrite(LED2, LOW);
        digitalWrite(redLED, LOW);
    }
}