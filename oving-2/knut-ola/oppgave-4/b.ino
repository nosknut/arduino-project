long randomTall;

void setup()
{
    Serial.begin(9600);
    randomSeed(analogRead(A0));
}

void loop()
{
    randomTall = random(1, 7);
    Serial.println(randomTall);
    if (randomTall == 6)
    {
        tone(10, 900, 200);
    }
    delay(200);
}