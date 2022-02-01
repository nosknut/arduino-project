long randNumber;
const int but = 2;
const unsigned long wait = 5000;
unsigned long prev = 0;
int lastBut = 0;

void setup()
{
  randomSeed(analogRead(A5));
  Serial.begin(9600);
  pinMode(but, INPUT);
}

void generate()
{
  int numbs[] = {0, 0, 0, 0, 0, 0};
  int j = 0;
  while (j < 10000)
  {
    int rnd = random(1, 7);
    numbs[rnd - 1]++;
    j++;
  }
  for (int i = 0; i <= 5; i++)
  {

    Serial.print("tallet ");
    Serial.print(i + 1);
    Serial.print(" ble generert ");
    Serial.print(numbs[i]);
    Serial.print(" ganger, og det tilsvarer ");
    Serial.print(numbs[i] / 100.0);
    Serial.println("%");
  }
  Serial.println("");
}

void loop()
{
  unsigned long currentTime = millis();
  int buton = digitalRead(but);

  if (buton == 1 && lastBut == 0)
  {
    if (currentTime - prev >= wait)
    {
      generate();
      prev = millis();
    }
    else
    {
      Serial.println("No data");
    }
  }
  lastBut = buton;
}
