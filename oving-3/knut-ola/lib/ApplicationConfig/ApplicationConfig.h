
struct ApplicationConfig
{
    const int baudRate = 9600;
    const int buttonDebounceTime = 50;
    const byte randomSeedPin = A0;
    const PlayerConfig player1 = {.ledPin = 12, .buttonPin = 13};
    const PlayerConfig player2 = {.ledPin = 7, .buttonPin = 6};
    const RgbLedConfig rgbLed = {.redPin = 8, .greenPin = 9};
    const byte buzzerPin = 11;
    const Range winnerBuzzerPitch = Range(750, 2000);
    const int looserBuzzerPitch = 220;
    const int roundCompletionAnnouncementDuration = 3000;
    // The frequency the LED's should blink after a win/loss
    const int fastBlinkFrequency = 5;
    // How long it should take for a green light to appear
    const Range roundTimeMs = Range(3000, 6000);
} const appConfig;
