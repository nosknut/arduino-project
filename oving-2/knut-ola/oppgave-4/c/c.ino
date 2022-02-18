class Buckets
{
private:
    int buckets[6];

public:
    Buckets()
    {
        reset();
    }

    void add(int value)
    {
        buckets[value] += 1;
    }

    int getCount(int value)
    {
        return buckets[value];
    }

    void reset()
    {
        for (int i = 0; i < 6; i++)
        {
            buckets[i] = 0;
        }
    }
};

const int analogSeedPin = A0;
Buckets buckets;

void setup()
{
    Serial.begin(9600);
    pinMode(analogSeedPin, INPUT);
    randomSeed(analogRead(analogSeedPin));
}

void loop()
{
    buckets.reset();
    int sampleSize = 10000;
    Serial.println("Sampling ...");
    for (int i = 0; i < sampleSize; i++)
    {
        buckets.add(random(0, 6));
    }
    for (int i = 0; i < 6; i++)
    {
        int count = buckets.getCount(i);
        Serial.print("The number ");
        Serial.print(i + 1);
        Serial.print(" was generated ");
        Serial.print(count);
        Serial.print(" times. That is ");
        Serial.print((((double)count) / sampleSize) * 100);
        Serial.println("%");
    }
    delay(1000);
}
