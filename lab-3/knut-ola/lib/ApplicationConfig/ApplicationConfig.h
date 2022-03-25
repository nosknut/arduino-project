
struct ApplicationConfig
{
    const int baudRate = 9600;
    const int buttonDebounceTime = 50;
    const byte randomSeedPin = A0;
    // NB! Update this with state.players
    const int numPlayers = 2;
    const PlayerConfig player1 = {.ledPin = 8, .buttonPin = 3};
    const PlayerConfig player2 = {.ledPin = 7, .buttonPin = 2};
    const RgbLedConfig rgbLed = {.redPin = 10, .greenPin = 9, .bluePin = 12};
    const byte buzzerPin = 11;
    const Range winnerBuzzerPitch = Range(750, 2000);
    const int winningPoints = 10;
    // 30% should be written as 0.3
    const float trickRoundProbability = 3.0 / 10.0;
    const int trickRoundDuration = 1000;
    const int looserBuzzerPitch = 220;
    const int roundCompletionAnnouncementDuration = 3000;
    // The frequency the LED's should blink after a win/loss
    const int fastBlinkFrequency = 5;
    // How long it should take for a green light to appear
    const Range roundTimeMs = Range(3000, 6000);
} const appConfig;
