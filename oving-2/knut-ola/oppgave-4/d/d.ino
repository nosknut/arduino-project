// https://github.com/bxparks/AceButton
#include <AceButton.h>
using namespace ace_button;

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
AceButton button(6, HIGH);
Buckets buckets;
bool generatingTestData = false;

void onClick(AceButton *button, uint8_t eventType, uint8_t buttonState)
{
    if (eventType == AceButton::kEventPressed)
    {
        if (generatingTestData)
        {
            Serial.println("No test data available yet");
        }
        else
        {
            generatingTestData = true;
        }
    }
}

void setup()
{
    Serial.begin(9600);
    pinMode(analogSeedPin, INPUT);
    pinMode(button.getPin(), INPUT_PULLUP);
    randomSeed(analogRead(analogSeedPin));
    button.setEventHandler(onClick);
}

void generateTestData()
{
    int sampleSize = 10000;
    Serial.println("Sampling ...");
    for (int i = 0; i < sampleSize; i++)
    {
        // Update button to trigger event handler during slow task
        button.check();
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
}

void loop()
{
    button.check();
    buckets.reset();
    if (generatingTestData)
    {
        generateTestData();
        generatingTestData = false;
    }
}
