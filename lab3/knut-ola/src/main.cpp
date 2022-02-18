#include <Arduino.h>
#include <ezButton.h>
#include <Ramp.h>

#include <RgbLedConfig.h>
#include <Range.h>
#include <PlayerConfig.h>
#include <ApplicationConfig.h>
#include <Timer.h>

class Player
{
public:
    const PlayerConfig playerConfig;
    ezButton button;
    Player(const PlayerConfig playerConfig) : playerConfig(playerConfig), button(ezButton(playerConfig.buttonPin))
    {
    }
    void setup()
    {
        pinMode(playerConfig.ledPin, OUTPUT);
        pinMode(playerConfig.buttonPin, INPUT_PULLUP);
        button.setDebounceTime(appConfig.buttonDebounceTime);
    }
};

struct ApplicationState
{
    Player player1 = Player(appConfig.player1);
    Player player2 = Player(appConfig.player2);
} state;

// Represents half of the period (a single on and off cycle) of a blink
int frequencyToHalfPeriodDelayTimeMs(const int frequency)
{
    return 1000 / frequency / 2;
}

// Pass the buzzer frequency through this to make a more interesting sound
// The function was arbitrarily choosen
int fancySoundFunction(const int frequency)
{
    return 2 * cos(frequency * 1.5 * sin(frequency));
}

void indicateWinner(Player winner)
{
    bool ledState = true;
    ramp buzzerRamp;
    const auto rampTargetValue = appConfig.winnerBuzzerPitch.maxValue - appConfig.winnerBuzzerPitch.minValue;

    Timer blinkTimer;
    const int blinkDelayTimeMs = frequencyToHalfPeriodDelayTimeMs(appConfig.fastBlinkFrequency);
    buzzerRamp.go(rampTargetValue, appConfig.roundCompletionAnnouncementDuration);
    for (; buzzerRamp.isRunning();)
    {
        buzzerRamp.update();
        const auto buzzerFrequency = buzzerRamp.getValue() + appConfig.winnerBuzzerPitch.minValue;
        tone(appConfig.buzzerPin, fancySoundFunction(buzzerFrequency));

        if (blinkTimer.loopWait(blinkDelayTimeMs))
        {
            ledState = !ledState;
            digitalWrite(winner.playerConfig.ledPin, ledState);
            digitalWrite(appConfig.rgbLed.greenPin, !ledState);
        }
    }

    digitalWrite(winner.playerConfig.ledPin, LOW);
    digitalWrite(appConfig.rgbLed.redPin, LOW);
    noTone(appConfig.buzzerPin);
}

void indicateLooser(Player looser)
{
    Timer blinkTimer;
    bool ledState = true;
    Timer announcementTimer;
    const int blinkDelayTimeMs = frequencyToHalfPeriodDelayTimeMs(appConfig.fastBlinkFrequency);
    announcementTimer.reset();
    while (!announcementTimer.loopWait(appConfig.roundCompletionAnnouncementDuration))
    {
        tone(appConfig.buzzerPin, appConfig.looserBuzzerPitch);

        if (blinkTimer.loopWait(blinkDelayTimeMs))
        {
            ledState = !ledState;
            digitalWrite(looser.playerConfig.ledPin, ledState);
            digitalWrite(appConfig.rgbLed.redPin, !ledState);
        }
    }
    digitalWrite(looser.playerConfig.ledPin, LOW);
    digitalWrite(appConfig.rgbLed.redPin, LOW);
    noTone(appConfig.buzzerPin);
}

int randomInRange(const Range range)
{
    return random(range.minValue, range.maxValue);
}

// Wait for both buttons to be released
// This is needed because the ezButton lib would
// remain pressed after the game finished and
// another game started.
// Adding a delay between games did not fix the issue.
void waitForButtonsToBeUnpressed()
{
    while (state.player1.button.isPressed() || state.player2.button.isPressed())
    {
        delay(1);
    }
}

void startGame()
{
    Timer roundTimer;
    Timer blinkTimer;
    const int roundTimeMs = randomInRange(appConfig.roundTimeMs);
    waitForButtonsToBeUnpressed();
    while (true)
    {
        state.player1.button.loop();
        state.player2.button.loop();
        const auto pressed1 = state.player1.button.isPressed();
        const auto pressed2 = state.player2.button.isPressed();
        if (!roundTimer.isFinished(roundTimeMs))
        {
            digitalWrite(appConfig.rgbLed.redPin, HIGH);
            digitalWrite(appConfig.rgbLed.greenPin, LOW);
            if (pressed1)
            {
                indicateLooser(state.player1);
                return;
            }
            else if (pressed2)
            {
                indicateLooser(state.player2);
                return;
            }
        }
        else
        {

            digitalWrite(appConfig.rgbLed.redPin, LOW);
            digitalWrite(appConfig.rgbLed.greenPin, HIGH);
            if (pressed1)
            {
                indicateWinner(state.player1);
                return;
            }
            else if (pressed2)
            {
                indicateWinner(state.player2);
                return;
            }
        }
    }
}

void setup()
{
    Serial.begin(appConfig.baudRate);
    randomSeed(analogRead(appConfig.randomSeedPin));
    state.player1.setup();
    state.player2.setup();
}

void loop()
{
    startGame();
}
