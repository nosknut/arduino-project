const int potmeterPin = A0;
const int ledPin = 11;

void setup()
{
    pinMode(potmeterPin, INPUT);
    pinMode(ledPin, OUTPUT);
    Serial.begin(9600);
}

int breathFunction(int x)
{
    // f(x) = e^kx - 1
    // f(255) = 255 = e^kx - 1
    // f(0) = 0 = e^kx - 1
    // k = log(256) / 255
    return exp((log(256) / 255) * x) - 1;
}

void pulseLed(int x)
{
    int value = breathFunction(x);
    analogWrite(ledPin, value);
    // Display the following in the Serial Plotter (not monitor)
    Serial.print(x);
    Serial.print(" ");
    Serial.println(value);
    delay(10);
}

void loop()
{
    for (int x = 0; x <= 255; x++)
    {
        pulseLed(x);
    }
    for (int x = 255; x >= 0; x--)
    {
        pulseLed(x);
    }
}
