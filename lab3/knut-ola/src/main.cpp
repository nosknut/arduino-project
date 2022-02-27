// The "//-LaTeX:SectionName;" anchors
// are for auto generated repors.
// See: ../report/report.tex
// NB!
// There can not be a /* type comment on the
// line directly below an anchor.

//-LaTeX:imports;
#include <Arduino.h>
#include <ezButton.h>
#include <Ramp.h>

#include <RgbLedConfig.h>
#include <Range.h>
#include <PlayerConfig.h>
#include <ApplicationConfig.h>
#include <Timer.h>
//-LaTeX:End_Section;

//-LaTeX:Player;
class Player
{
public:
    const PlayerConfig playerConfig;
    int points = 0;
    ezButton button;
    Player(const PlayerConfig playerConfig) : playerConfig(playerConfig), button(ezButton(playerConfig.buttonPin))
    {
    }

    void changePoints(int points)
    {
        this->points += points;
    }

    void resetPoints()
    {
        this->points = 0;
    }

    int getPoints()
    {
        return this->points;
    }

    void setup()
    {
        pinMode(playerConfig.ledPin, OUTPUT);
        pinMode(playerConfig.buttonPin, INPUT_PULLUP);
        button.setDebounceTime(appConfig.buttonDebounceTime);
    }
};
//-LaTeX:End_Section;

//-LaTeX:RgbLed;
class RgbLed
{
    RgbLedConfig rgbLedConfig;

public:
    RgbLed(const RgbLedConfig rgbLedConfig) : rgbLedConfig(rgbLedConfig)
    {
    }

    void red(bool state)
    {
        off();
        digitalWrite(rgbLedConfig.redPin, state);
    }

    void green(bool state)
    {
        off();
        digitalWrite(rgbLedConfig.greenPin, state);
    }

    void blue(bool state)
    {
        off();
        digitalWrite(rgbLedConfig.bluePin, state);
    }

    void off()
    {
        digitalWrite(rgbLedConfig.redPin, LOW);
        digitalWrite(rgbLedConfig.greenPin, LOW);
        digitalWrite(rgbLedConfig.bluePin, LOW);
    }

    void setup()
    {
        pinMode(rgbLedConfig.redPin, OUTPUT);
        pinMode(rgbLedConfig.greenPin, OUTPUT);
        pinMode(rgbLedConfig.bluePin, OUTPUT);
    }
};
//-LaTeX:End_Section;

//-LaTeX:SerialCommand;
enum class SerialCommand
{
    START = 's',
    STOP = 'q',
    RESET = 'r',
    HELP = 'h',
};
//-LaTeX:End_Section;

//-LaTeX:GameState;
enum class GameState
{
    IDLE,
    RUNNING,
};
//-LaTeX:End_Section;

//-LaTeX:ApplicationState;
struct ApplicationState
{
    // NB! Update this with appConfig.numPlayers
    Player players[2] = {
        Player(appConfig.player1),
        Player(appConfig.player2)};
    RgbLed rgbLed = RgbLed(appConfig.rgbLed);
    GameState gameState = GameState::IDLE;
} state;
//-LaTeX:End_Section;

//-LaTeX:SerialMessages;
void printHelp()
{
    Serial.println("------------------------------------------------------");
    Serial.println("Available commands:");
    Serial.println("");
    Serial.println("s - Start the game");
    Serial.println("q - Stop the game and show the winner");
    Serial.println("r - Reset the game/score");
    Serial.println("h - Show this message again");
    Serial.println("------------------------------------------------------");
}

void printWinner(Player &winner, int playerIndex)
{
    Serial.println("------------------------------------------------------");
    Serial.println("Player " + String(playerIndex + 1) + " wins!");
    Serial.println("Score: " + String(winner.getPoints()));
    Serial.println("------------------------------------------------------");
    printHelp();
}
//-LaTeX:End_Section;

//-LaTeX:Reset;
void resetIo()
{
    state.rgbLed.off();
    noTone(appConfig.buzzerPin);
    for (Player &player : state.players)
    {
        digitalWrite(player.playerConfig.ledPin, LOW);
    }
}

void resetGame()
{
    state.gameState = GameState::IDLE;
    for (Player &player : state.players)
    {
        player.resetPoints();
    }
}
//-LaTeX:End_Section;

//-LaTeX:getBestPlayerIndex;
int getBestPlayerIndex()
{
    int bestPlayerIndex = 0;
    for (int i = 1; i < appConfig.numPlayers; i++)
    {
        Player &player = state.players[i];
        Player &bestPlayer = state.players[bestPlayerIndex];
        if (player.getPoints() > bestPlayer.getPoints())
        {
            bestPlayerIndex = i;
        }
    }
    return bestPlayerIndex;
}
//-LaTeX:End_Section;

//-LaTeX:updateGameState;

/**
 * @brief Checks for serial commands
 * @return true if game should keep running
 */
bool updateGameState()
{
    while (Serial.available())
    {
        const char input = toLowerCase(Serial.read());
        const auto command = static_cast<SerialCommand>(input);
        switch (command)
        {
        case SerialCommand::START:
            resetGame();
            state.gameState = GameState::RUNNING;
            Serial.println("Game started!");
            break;
        case SerialCommand::STOP:
        {
            Serial.println("Game stopped!");
            int bestPlayerIndex = getBestPlayerIndex();
            printWinner(state.players[bestPlayerIndex], bestPlayerIndex);
            state.gameState = GameState::IDLE;
        }
        break;
        case SerialCommand::RESET:
            resetGame();
            Serial.println("Everything has been reset!");
            break;
        case SerialCommand::HELP:
            printHelp();
            break;
        default:
            break;
        }
    }
    if (state.gameState == GameState::RUNNING)
    {
        int bestPlayerIndex = getBestPlayerIndex();
        Player &bestPlayer = state.players[bestPlayerIndex];
        if (bestPlayer.getPoints() >= appConfig.winningPoints)
        {
            Serial.println("Player " + String(bestPlayerIndex + 1) + " has the highest score: " + String(bestPlayer.getPoints()));
            printWinner(bestPlayer, bestPlayerIndex);
            state.gameState = GameState::IDLE;
        }
    }
    if (state.gameState == GameState::IDLE)
    {
        resetIo();
    }
    return state.gameState == GameState::RUNNING;
}
//-LaTeX:End_Section;

//-LaTeX:fancySoundFunction;
// Represents half of the period (a single on and off cycle) of a blink
int frequencyToHalfPeriodDelayTimeMs(const int frequency)
{
    return 1000 / frequency / 2;
}

// Pass the buzzer frequency through this to make a more interesting sound
// The function was arbitrarily choosen
int fancySoundFunction(const int frequency)
{
    long x = millis() / 20;
    return frequency + 500 * cos(x + sin(x));
}
//-LaTeX:End_Section;

//-LaTeX:printPoints;
void printPoints()
{
    Serial.print("Score: ");
    for (int i = 0; i < appConfig.numPlayers; i++)
    {
        Player &player = state.players[i];
        if (i != 0)
        {
            Serial.print(", ");
        }
        Serial.print(player.getPoints());
    }
    Serial.println("");
}
//-LaTeX:End_Section;

//-LaTeX:indicateWinner;
void indicateWinner(Player &winner)
{
    bool ledState = true;
    ramp buzzerRamp;
    const auto rampTargetValue = appConfig.winnerBuzzerPitch.maxValue - appConfig.winnerBuzzerPitch.minValue;

    Timer blinkTimer;
    const int blinkDelayTimeMs = frequencyToHalfPeriodDelayTimeMs(appConfig.fastBlinkFrequency);
    buzzerRamp.go(rampTargetValue, appConfig.roundCompletionAnnouncementDuration);
    for (; buzzerRamp.isRunning();)
    {
        if (!updateGameState())
        {
            return;
        }
        buzzerRamp.update();
        const auto buzzerFrequency = buzzerRamp.getValue() + appConfig.winnerBuzzerPitch.minValue;
        tone(appConfig.buzzerPin, fancySoundFunction(buzzerFrequency));

        if (blinkTimer.loopWait(blinkDelayTimeMs))
        {
            ledState = !ledState;
            digitalWrite(winner.playerConfig.ledPin, ledState);
            state.rgbLed.green(!ledState);
        }
    }

    resetIo();
}
//-LaTeX:End_Section;

//-LaTeX:indicateLooser;
void indicateLooser(Player &looser)
{
    Timer blinkTimer;
    bool ledState = true;
    Timer announcementTimer;
    const int blinkDelayTimeMs = frequencyToHalfPeriodDelayTimeMs(appConfig.fastBlinkFrequency);
    announcementTimer.reset();
    while (!announcementTimer.loopWait(appConfig.roundCompletionAnnouncementDuration))
    {
        if (!updateGameState())
        {
            return;
        }
        tone(appConfig.buzzerPin, appConfig.looserBuzzerPitch);

        if (blinkTimer.loopWait(blinkDelayTimeMs))
        {
            ledState = !ledState;
            digitalWrite(looser.playerConfig.ledPin, ledState);
            state.rgbLed.red(!ledState);
        }
    }
    resetIo();
}
//-LaTeX:End_Section;

//-LaTeX:waitForButtonsToBeUnpressed;

// Wait for both buttons to be released
// This is needed because the ezButton lib would
// remain pressed after the game finished and
// another game started.
// Adding a delay between games did not fix the issue.
void waitForButtonsToBeUnpressed()
{
    while (true)
    {
        bool anyButtonIsPressed = false;
        for (Player &player : state.players)
        {
            player.button.loop();
            if (player.button.isPressed())
            {
                anyButtonIsPressed = true;
                break;
            }
        }
        if (!anyButtonIsPressed)
        {
            break;
        }
        delay(1);
    }
}
//-LaTeX:End_Section;

//-LaTeX:startGame;
int randomInRange(const Range range)
{
    return random(range.minValue, range.maxValue);
}

void startGame()
{
    Timer roundTimer;
    Timer trickRoundTimer;
    Timer blinkTimer;
    const int roundTimeMs = randomInRange(appConfig.roundTimeMs);
    const int trickRoundTimeMs = appConfig.trickRoundDuration + roundTimeMs;
    const bool trickRound = random(0, 100) <= (appConfig.trickRoundProbability * 100);
    waitForButtonsToBeUnpressed();
    while (true)
    {
        if (!updateGameState())
        {
            return;
        }

        const bool roundFinished = roundTimer.isFinished(roundTimeMs);
        const bool trickRoundFinished = trickRoundTimer.isFinished(trickRoundTimeMs);

        if (roundFinished)
        {
            if (trickRound)
            {
                state.rgbLed.blue(true);
            }
            else
            {
                state.rgbLed.green(true);
            }
        }
        else
        {
            state.rgbLed.red(true);
        }

        for (Player &player : state.players)
        {
            player.button.loop();
            const bool buttonPressed = player.button.isPressed();
            if (buttonPressed)
            {
                if (roundFinished)
                {
                    // Triggerhappy
                    if (trickRound)
                    {

                        player.changePoints(-2);
                        printPoints();
                        indicateLooser(player);
                        return;
                    }
                    // Winner
                    else
                    {
                        int reactionTimeMs = roundTimer.getElapsedTime() - roundTimeMs;
                        float reactionTimeSec = reactionTimeMs / 1000.0;
                        // Add a small amount of time to avoid dividing by zero
                        int points = 1 / (reactionTimeSec + 0.000000001);
                        Serial.println("Reaction time: " + String(reactionTimeSec) + " seconds");
                        player.changePoints(points);
                        printPoints();
                        indicateWinner(player);
                        return;
                    }
                }
                else
                // Tricked
                {
                    player.changePoints(-1);
                    printPoints();
                    indicateLooser(player);
                    return;
                }
            }
            if (trickRound && trickRoundFinished)
            {
                Serial.println("Time expired");
                return;
            }
        }
    }
}
//-LaTeX:End_Section;

//-LaTeX:setup;
void setup()
{
    Serial.begin(appConfig.baudRate);
    randomSeed(analogRead(appConfig.randomSeedPin));
    for (Player &player : state.players)
    {
        player.setup();
    }
    state.rgbLed.setup();
    pinMode(appConfig.buzzerPin, OUTPUT);
    printHelp();
}
//-LaTeX:End_Section;

//-LaTeX:loop;
void loop()
{
    if (updateGameState())
    {
        startGame();
    }
    else
    {
        delay(1);
    }
}
//-LaTeX:End_Section;
