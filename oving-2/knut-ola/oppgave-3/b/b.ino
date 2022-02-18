const int sensorPin = A0;
const int ledPin = 11;

int lowerLimit = 1023;
int upperLimit = 0;

void calibratePhotoresistorLimits(int value)
{
    if (value > upperLimit)
    {
        upperLimit = value;
    }
    if (value < lowerLimit)
    {
        lowerLimit = value;
    }
}

void setup()
{
    pinMode(sensorPin, INPUT);
    pinMode(ledPin, OUTPUT);
    Serial.begin(9600);
}

void loop()
{
    int sensorValue = analogRead(sensorPin);
    // Computes limits on the fly. These values should be st
    calibratePhotoresistorLimits(sensorValue);
    int value = map(sensorValue, lowerLimit, upperLimit, 0, 255);
    analogWrite(ledPin, value);
    // Display the following in the Serial Plotter (not monitor)
    Serial.print("(");
    Serial.print(lowerLimit);
    Serial.print(" < ");
    Serial.print(sensorValue);
    Serial.print(" < ");
    Serial.print(upperLimit);
    Serial.print(")");
    Serial.print(" -> ");
    Serial.println(value);
    delay(10);
}
