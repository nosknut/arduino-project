const int potmeterPin = A0;

void setup()
{
    pinMode(potmeterPin, INPUT);
    Serial.begin(9600);
}

void loop()
{
    Serial.println(analogRead(potmeterPin));
}

/*
Potmeter range: 0-1023
When connected to GND: 0
When connected to VCC: 1023
*/