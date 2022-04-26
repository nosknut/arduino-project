//lager variabler så det er enklere å endre koden
const int sensorPin = A0;
//endrer disse til float, så jeg får R (hvis ikke 0)
int led = 10;
int sensorValue;
int sensorValueRange;

void setup()
{
    Serial.begin(9600);
}

void loop()
{
    sensorValue = analogRead(sensorPin);

    // Fant øvre og nedre grense til 450 (lysest)
    // og 982 (mørkest) på photoresistor
    sensorValueRange = map(sensorValue, 450, 982, 0, 255);
    analogWrite(led, sensorValueRange);
}