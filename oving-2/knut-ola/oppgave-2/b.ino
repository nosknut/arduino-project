//lager variabler så det er enklere å endre koden
const int sensorPin = A0;
//endrer disse til float, så jeg får R (hvis ikke 0)
int sensorValue;
int sensorValueVoltage;
int lastSensorValueVoltage;
int i = 0;

const char *meld[5] = {"1V, data: ", "2V, data: ", "3V, data: ",
                       "4V, data: ", "5V, data: "};

void setup()
{
    Serial.begin(9600);
}

void loop()
{
    sensorValue = analogRead(sensorPin);

    /* Bruker map for enkel kode og sikre
     riktige verdier som brukes videre i array*/
    sensorValueVoltage = map(sensorValue, 0, 1023, 0, 4);

    if (i == 0)
    { // forsikrer at det blir printet noe første gang
        Serial.println(meld[sensorValueVoltage]);
        i++;
    }

    // printer bare hvis verdien endrer seg
    if (sensorValueVoltage != lastSensorValueVoltage)
    {
        Serial.println(meld[sensorValueVoltage]);
    }

    // lagrer den forje verdien, så vi kan se om det er endring
    lastSensorValueVoltage = sensorValueVoltage;
}