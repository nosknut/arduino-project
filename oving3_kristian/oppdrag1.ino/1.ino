int redLED = 10;    //RGB-rød
int greenLED = 9;   //RGB-grønn
int buzzerPin = 11; //Buzzer

int LED1 = 6; //Spiller 1 - LED
int LED2 = 7; //Spiller 2	- LED
int SW1 = 3;  //Spiller 1 - knapp
int SW2 = 4;  //Spiller 2 - knapp

int winner = 0;       //Initialiserer vinnerindikatoren
int winnerBeep = 750; //Buzzer-pitchen for vinnerfanfaren
int fault = 0;        //Initialiserer feilindikatoren
int faultBeep = 200;  //Buzzer-pitchen for feillyden

unsigned long wait = 0;
unsigned long now = 0;

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000); // Wait for 1000 millisecond(s)
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000); // Wait for 1000 millisecond(s)
}