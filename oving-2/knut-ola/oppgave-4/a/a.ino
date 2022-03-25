const int analogSeedPin = A0;

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
    Serial.println(random(1, 7));
    sleep(1000);
}
