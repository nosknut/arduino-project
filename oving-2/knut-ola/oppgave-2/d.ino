//lager variabler så det er enklere å endre koden
const int sensorPin = A0;
//endrer disse til float, så jeg får R (hvis ikke 0)
int sensorValue;
int sensorValueRange;

void setup()
{
    Serial.begin(9600);
}

void loop()
{
    sensorValue = analogRead(sensorPin);

    // Bruker map som gir meg verdier (0,1,2,3) fra verdiene 0->1023
    sensorValueRange = map(sensorValue, 0, 1023, 0, 3);

    Serial.println("Range");
    Serial.print(sensorValueRange);
}