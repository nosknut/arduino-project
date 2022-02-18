const int potmeterPin = A0;

void setup()
{
    pinMode(potmeterPin, INPUT);
    Serial.begin(9600);
}

/*
Same as the builtin arduino map() function
https://www.arduino.cc/reference/en/language/functions/math/map/
but supports decimals
*/
double mapFloat(int value, int min, int max, int newMin, int newMax, int numDecimals)
{
    double multiplier = pow(10, numDecimals);
    return map(value, min, max, newMin * multiplier, newMax * multiplier) / multiplier;
}

void loop()
{
    int value = analogRead(potmeterPin);
    int numVoltageDecimals = 1;
    int maxVoltage = 5;
    int resistorSize = 10e3;
    float voltage = mapFloat(value, 0, 1023, 0, maxVoltage, numVoltageDecimals);
    // Pro gamer move:
    float resistance = map(value, 0, 1023, 0, resistorSize);
    Serial.print(voltage, numVoltageDecimals);
    Serial.print(" V; ");
    Serial.print((int)resistance);
    Serial.println(" Ohm; ");
}

/*
Potmeter range: 0-5
We measure the voltage drop over
the resistor between the A0 pin and GND
*/
