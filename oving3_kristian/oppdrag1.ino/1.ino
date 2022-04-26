#include <ezButton.h>

// variabler
int redLED = 10;    // RGB-rød
int greenLED = 9;   // RGB-grønn
int buzzerPin = 11; // Buzzer

int LED1 = 6;     // Spiller 1 - LED
int LED2 = 7;     // Spiller 2	- LED
ezButton SW1 = 3; // Spiller 1 - knapp
ezButton SW2 = 4; // Spiller 2 - knapp

// random variabler
long randomTall;
const int seedPin = A0;
long currentTime = 0; // tidsvariabel

void setup()
{
    // starter serial og definerer LED som OUTPUT
    Serial.begin(9600);
    pinMode(redLED, OUTPUT);
    pinMode(greenLED, OUTPUT);
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    // setter dbounce
    SW1.setDebounceTime(50);
    SW2.setDebounceTime(50);

    // random seed og tall
    randomSeed(analogRead(seedPin));
    randomTall = random(3, 7);
}

// funksjon for vinnerlyd, tar inn LED-pinne variabel
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

// feil lyd funksjon, tar inn LED-pinne
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

// reset funksjon som lager nytt random tall og resetter lys, tid og lag
void resetFunksjon()
{
    digitalWrite(redLED, LOW);
    digitalWrite(greenLED, LOW);
    randomTall = random(1, 7);
    currentTime = millis();
}

// starter button-loops og setter debounce
void buttonStart()
{
    SW1.loop();
    SW2.loop();
}

void loop()
{
    buttonStart();
    // modifiserer tidsvariabelen "time" med variabel "currentTime"
    long time = millis() - currentTime;
    // definerer tiden for rød og grønn LED
    const long timeRedLed = (randomTall * 1000);
    const long timeGreenLed = timeRedLed + 1000;

    // tester om noen trykker mens tiden er mindre enn "timeRedLed"
    if (time < timeRedLed)
    {
        digitalWrite(redLED, HIGH);  // setter greenLed high
        digitalWrite(greenLED, LOW); // setter redLed low
        Serial.println("RedLedLyser");
        // hvis noen trykker i løkken, kjøres feilLyd til LED 1 eller 2
        if (SW1.isPressed())
        {
            feilLyd(LED1);
        }
        else if (SW2.isPressed())

        {
            feilLyd(LED2);
        }
    }
    // hvis tid er større enn redled men mindre enn green led
    //  sett farge til grønn og se etter knappetrykk
    if (time >= timeRedLed && time <= timeGreenLed)
    {
        digitalWrite(redLED, LOW);
        digitalWrite(greenLED, HIGH);
        Serial.print("GreenLedLyser");

        if (SW1.isPressed())
        {
            vinnerFanfaren(LED1);
        }
        else if (SW2.isPressed())
        {
            vinnerFanfaren(LED2);
        }
    }
    // resetter tid og lager nytt randomTall
    if (time > timeGreenLed)
    {
        resetFunksjon();
        delay(1000);
    }
}