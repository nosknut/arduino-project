#include <ezButton.h>

// variabler
int redLED = 10;    // RGB-rød
int greenLED = 9;   // RGB-grønn
int blueLED = 8;    // RGB-blå
int buzzerPin = 11; // Buzzer

int LED1 = 6;     // Spiller 1 - LED
int LED2 = 7;     // Spiller 2	- LED
ezButton SW1 = 3; // Spiller 1 - knapp
ezButton SW2 = 4; // Spiller 2 - knapp

// random variabler
long randomTall;
long randomTallBlue;
const int seedPin = A0;
long currentTime = 0; // tidsvariabel

// array for spillpoeng
int points[2] = {0, 0}; // posisjon 0 = spiller_1, posisjon 1 = spiller_2

enum class GameState
{
    RESET,
    RUNNING,
    DONE,
    IDLE
};

// switch funksjonalitet:
char inByte; // variabel switch
GameState gameState = GameState::IDLE;

void setup()
{
    // starter serial og definerer LED som OUTPUT
    Serial.begin(9600);
    pinMode(redLED, OUTPUT);
    pinMode(greenLED, OUTPUT);
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);

    // random seed og tall
    randomSeed(analogRead(seedPin));
    randomTall = random(3, 7);
    randomTallBlue = random(1, 11);

    // setter debounce
    SW1.setDebounceTime(50);
    SW2.setDebounceTime(50);

    // printer hvordan styre program:
    Serial.println("Med dette programmet kan du styre 3 LED's");
    Serial.println("S - starter nytt spill");
    Serial.println("Q - avslutter spillet, viser poeng");
    Serial.println("R - resetter spillet");
}

void vinnerFanfaren(int ledPin)
// funksjon for vinnerlyd, tar inn LED-pinne variabel
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

char charToLowerCase(char input)
{
    String lowerCaseInput = String(input);
    lowerCaseInput.toLowerCase();
    return lowerCaseInput[0];
}

void switchFunksjon(char input)
{
    switch (charToLowerCase(input))
    {
    case 'S':
    case 's': // sjekker både stor og liten "s"
        Serial.print("knappetrykk s");
        resetFunksjon();
        gameState = GameState::RUNNING;
        break;
    case 'Q':
    case 'q': // sjekker både stor og liten "q"
        Serial.print("knappetrykk q");
        gameState = GameState::DONE;
        break;
    case 'R':
    case 'r': // sjekker både stor og liten "r"
        Serial.print("knappetrykk r");
        resetFunksjon();
        gameState = GameState::RESET;
        break;

    default:
        Serial.println("Ugyldig kommando!");
    }
}
// printer instruksjoner med en valgt delay
void printInstructions(int delayTime)
{
    if (millis() % delayTime == 0)
    {
        // printer hvordan styre program:
        Serial.println("Med dette programmet kan du styre 3 LED's");
        Serial.println("S - starter nytt spill");
        Serial.println("Q - avslutter spillet, viser poeng");
        Serial.println("R - resetter spillet");
    }
}

// selve spill-loop
void gameLoop()
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
        if (randomTallBlue > 7)
        {
            digitalWrite(blueLED, HIGH);
        }
        else
        {
            digitalWrite(redLED, HIGH); // setter greenLed high
        }
        digitalWrite(greenLED, LOW); // setter redLed low
        Serial.println("RedLedLyser");
        // hvis noen trykker i løkken, kjøres feilLyd til LED 1 eller 2
        if (SW1.isPressed())
        {
            points[0]--;
            feilLyd(LED1);
            resetFunksjon();
            return;
        }
        if (SW2.isPressed())
        {
            points[1]--;
            feilLyd(LED2);
            resetFunksjon();
            return;
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
            points[0]++;
            vinnerFanfaren(LED1);
            resetFunksjon();
            return;
        }
        if (SW2.isPressed())
        {
            points[1]++;
            vinnerFanfaren(LED2);
            resetFunksjon();
            return;
        }
    }
    // resetter tid og lager nytt randomTall
    if (time > timeGreenLed)
    {
        resetFunksjon();
        delay(1000);
    }
}

void endGame()
{
    Serial.println("Spillet er ferdig!");
    Serial.println("Spiller ");
    if (points[0] > points[1])
    {
        Serial.print("1 vant!!!!");
        Serial.println("Spiller 1 fikk ");
        Serial.print(points[0]);
    }
    if (points[0] < points[1])
    {
        Serial.print("2 vant!!!!");
        Serial.println("Spiller 2 fikk ");
        Serial.print(points[1]);
    }
    printInstructions(2000); // passer på at det ikke printer for ofte
}

void resetGame()
{
    points[0] = 0;           // setter spiller_1 poeng == 0
    points[1] = 0;           // setter spiller_2 poeng == 0
    printInstructions(2000); // passer på at det ikke printer for ofte
}

void loop()
{
    // henter serialverdi hvis den eksisterer:
    if (Serial.available() > 0)
    {
        inByte = Serial.read(); // ja - les det inn i inByte
        switchFunksjon(inByte);
    }

    // sjekker om noen har fått 5 poeng
    if (points[0] == 5 || points[1] == 5)
    {
        gameState = GameState::DONE;
    }

    //----If-settninger for å teste GameState----
    if (gameState == GameState::RUNNING)
    {
        gameLoop();
    }
    if (gameState == GameState::DONE)
    {
        endGame();
    }
    if (gameState == GameState::RESET)
    {
        resetGame();
    }
}