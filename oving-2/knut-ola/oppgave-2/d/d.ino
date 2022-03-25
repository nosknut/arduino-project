const int potmeterPin = A0;

void setup()
{
    pinMode(potmeterPin, INPUT);
    Serial.begin(9600);
}

int getRange(int value, int range, bool reverse)
{
    int valueInRange = map(value, 0, 1023, 0, range);
    return reverse ? range - valueInRange : valueInRange;
}

void loop()
{
    int value = analogRead(potmeterPin);
    Serial.print(" range ");
    Serial.println(getRange(value, 3, true));
    delay(100);
}
