int random;

void setup()
{
    Serial.begin(9600);
}

void loop()
{
    random = rand() % 6 + 1;
    Serial.println(random);
    delay(1000);
}