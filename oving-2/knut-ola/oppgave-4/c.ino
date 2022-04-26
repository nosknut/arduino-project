long randomTall;
// bruker array for å telle de forskjellige tallene
// bruker index 0 for tall 1, index 1 for tall 2 osv.
int arrayTall[6] = {
    0,
    0,
    0,
    0,
    0,
    0,
};

void setup()
{
    Serial.begin(9600);
    randomSeed(analogRead(A0));
}

// ettersom vi ikke har lært å finne lengde på array
// hardkoder jeg det inn for nå
int testTall(int inputTall, int inputArray)
{
    for (int i = 0; i < 6; i++)
    {
        if (inputTall == inputArray[i])
        {
            inputArray[i] += 1;
        }
    }
    return inputArray;
}

void loop()
{
    for (int i = 0; i < 10001; i++)
    {
        randomTall = random(1, 7);
        arrayTall = testTall(randomTall, arrayTall);
    }
    Serial.println(arrayTall);
}