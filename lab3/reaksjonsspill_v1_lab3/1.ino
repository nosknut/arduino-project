#include <ezButton.h>

// variabler
int redLED = 10;    // RGB-rød
int greenLED = 9;   // RGB-grønn
int blueLED = 8;    // RGB-blå
int buzzerPin = 11; // Buzzer

int LED1 = 6;          // Spiller 1 - LED
int LED2 = 7;          // Spiller 2	- LED
ezButton BUTTON_1 = 3; // Spiller 1 - knapp
ezButton BUTTON_2 = 4; // Spiller 2 - knapp

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

enum class PrintCommand
{
    RESET,
    DONE,
    IDLE
};

// switch funksjonalitet:
char inByte; // variabel switch
GameState gameState = GameState::IDLE;
PrintCommand printCommand = PrintCommand::IDLE;

void setup()
{
    // starter serial og definerer LED som OUTPUT
    Serial.begin(9600);
    pinMode(redLED, OUTPUT);
    pinMode(greenLED, OUTPUT);
    pinMode(blueLED, OUTPUT);
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);

    // random seed og tall
    randomSeed(analogRead(seedPin));
    randomTall = random(3, 7);
    randomTallBlue = random(1, 11);

    // setter debounce
    // BUTTON_1.setDebounceTime(50);
    // BUTTON_2.setDebounceTime(50);

    // printer hvordan styre program:
    Serial.println("Med dette programmet kan du styre med tre bokstaver:");
    Serial.println("S - starter nytt spill");
    Serial.println("Q - avslutter spillet, viser poeng");
    Serial.println("R - resetter spillet\n");
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
    digitalWrite(blueLED, LOW);
    randomTall = random(1, 7);
    randomTallBlue = random(1, 11);
    currentTime = millis();
}

// starter button-loops og setter debounce
void buttonStart()
{
    BUTTON_1.loop();
    BUTTON_2.loop();
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
        resetFunksjon();
        gameState = GameState::RUNNING;
        break;
    case 'Q':
    case 'q': // sjekker både stor og liten "q"
        gameState = GameState::DONE;
        printCommand = PrintCommand::DONE;
        break;
    case 'R':
    case 'r': // sjekker både stor og liten "r"
        resetFunksjon();
        gameState = GameState::RESET;
        printCommand = PrintCommand::RESET;
        break;
    }
}
// printer instruksjoner med en valgt delay
void printInstructions()
{
    // printer hvordan styre program:
    Serial.println("Med dette programmet kan du styre med tre bokstaver:");
    Serial.println("S - starter nytt spill");
    Serial.println("Q - avslutter spillet, viser poeng");
    Serial.println("R - resetter spillet\n");
}

// printer game stats
void gameStats()
{
    Serial.print("Spiller 1 har: ");
    Serial.println(points[0]);
    Serial.print("Spiller 2 har: ");
    Serial.println(points[1]);
}

// tar inn tidLED og hvilken spiller (1 indeksert, funksjonen gjør om til 0 selv)
void poengFunksjon(float timeLed, float gameTime, int player)
{
    player--; // ettersom array 0 indekserer
    float diff = timeLed - gameTime;
    float responseTime = 1000 - diff; // 1000 = greenLedTid
    if (digitalRead(blueLED))
    {                        // hvis trykk når blå:
        points[player] -= 2; // - 2 poeng
    }
    else
    {
        if (responseTime < 200)
        {
            points[player] += 3;
        }
        else if (responseTime < 400)
        {
            points[player] += 2;
        }
        else
        {
            points[player]++;
        }
    }
    Serial.print(responseTime);
    Serial.println("ms");
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
        digitalWrite(redLED, HIGH);  // setter greenLed high
        digitalWrite(greenLED, LOW); // setter redLed low
        // hvis noen trykker i løkken, kjøres feilLyd til LED 1 eller 2
        if (BUTTON_1.isPressed())
        {
            Serial.println("player one klikk");
            points[0]--;
            feilLyd(LED1);
            resetFunksjon();
            gameStats();
            return;
        }
        if (BUTTON_2.isPressed())
        {
            Serial.println("player two klikk");
            points[1]--;
            feilLyd(LED2);
            resetFunksjon();
            gameStats();
            return;
        }
    }
    // hvis tid er større enn redled men mindre enn green led
    //  sett farge til grønn og se etter knappetrykk
    if (time >= timeRedLed && time <= timeGreenLed)
    {
        digitalWrite(redLED, LOW);
        if (randomTallBlue > 7)
        {
            digitalWrite(blueLED, HIGH);
        }
        else
        {
            digitalWrite(greenLED, HIGH);
        }
        //-----knapp 1--------
        if (BUTTON_1.isPressed())
        {
            Serial.println("player one klikk");
            poengFunksjon(timeGreenLed, time, 1);
            vinnerFanfaren(LED1);
            resetFunksjon();
            gameStats();
            return;
        }
        //-----Knapp 2--------
        if (BUTTON_2.isPressed())
        {
            Serial.println("player two klikk");
            poengFunksjon(timeGreenLed, time, 2);
            vinnerFanfaren(LED2);
            resetFunksjon();
            gameStats();
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
    Serial.print("Spiller ");
    if (points[0] > points[1])
    {
        Serial.print("1 vant!!!!");
        Serial.print("Spiller 1 fikk ");
        Serial.println(points[0]);
    }
    if (points[0] < points[1])
    {
        Serial.print("2 vant!!!!");
        Serial.print("Spiller 2 fikk ");
        Serial.println(points[1]);
    }
    printInstructions(); // passer på at det ikke printer for ofte
    // resetter poeng etter print
    points[0] = 0;
    points[1] = 0;
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
    if (points[0] >= 5 || points[1] >= 5)
    {
        gameState = GameState::DONE;
        printCommand = PrintCommand::DONE;
    }

    //----If-settninger for å teste GameState----
    //-----Game: RUNNING-------
    if (gameState == GameState::RUNNING)
    {
        gameLoop();
    }
    //-----Game: DONE--------
    if (gameState == GameState::DONE)
    {
        if (printCommand == PrintCommand::DONE)
        {
            endGame();
            printCommand = PrintCommand::IDLE;
        }
    }
    //-----Game: RESET-------
    if (gameState == GameState::RESET)
    {
        if (printCommand == PrintCommand::RESET)
        {
            points[0] = 0;
            points[1] = 0;
            Serial.print("Game er resatt, følg instruksjoner: ");
            printInstructions();
            printCommand = PrintCommand::IDLE;
        }
    }
}