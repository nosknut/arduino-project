const int analogSeedPin = A0;
const int buzzerPin = 9;

void setup()
{
    Serial.begin(9600);
    pinMode(analogSeedPin, INPUT);
    randomSeed(analogRead(analogSeedPin));
}

void sleep(int ms)
{
    unsigned long start = millis();
    while ((millis() - start) < ms)
    {
        // Block code execution until the specified time has passed.
    }
}

void loop()
{
    if (random(1, 7) == 6)
    {
        tone(buzzerPin, 1000, 200);
        imperialMarch();
    }
    sleep(1000);
}

// https://lochnerweb.de/imperial-march-in-arduino
#define a 440
#define f 349
#define c2 523
#define e2 659
#define f2 698

#define gn 800
#define hn gn / 2
#define vn gn / 4
#define an gn / 8

#define pin buzzerPin

void imperialMarch()

{

    tone(pin, a, hn);
    delay(hn + 100);
    tone(pin, a, hn);
    delay(hn + 100);

    tone(pin, a, hn);
    delay(hn + 100);
    tone(pin, f, vn + an);
    delay(vn + 100 + an);
    tone(pin, c2, an);
    delay(an + 100);

    tone(pin, a, hn);
    delay(hn + 100);
    tone(pin, f, vn + an);
    delay(vn + 100 + an);
    tone(pin, c2, an);
    delay(an + 100);

    tone(pin, a, gn);
    delay(gn + 200);

    tone(pin, e2, hn);
    delay(hn + 100);
    tone(pin, e2, hn);
    delay(hn + 100);

    tone(pin, e2, hn);
    delay(hn + 100);
    tone(pin, f2, vn + an);
    delay(vn + 100 + an);
    tone(pin, c2, an);
    delay(an + 100);

    tone(pin, a, hn);
    delay(hn + 100);
    tone(pin, f, vn + an);
    delay(vn + 100 + an);
    tone(pin, c2, an);
    delay(an + 100);

    tone(pin, a, gn);
    delay(gn + 200);
}