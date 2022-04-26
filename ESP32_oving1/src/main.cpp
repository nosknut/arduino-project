#include <Arduino.h>

// inneholder OPPGAVE1 - 4
enum class OppgaveStatus
{
  OPPGAVE1,
  OPPGAVE2,
  OPPGAVE3,
  OPPGAVE4
};

//-----VELG HVILKEN OPPGVAVE SOM SKAL KJØRES------
OppgaveStatus oppgave = OppgaveStatus::OPPGAVE4;

// ledcSetup(PWM - kanal(0 - 15), PWM.frekvens, PWM - oppløsning(i bit));

const int LED = 32;

// channels
const int channelLED = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(LED, OUTPUT);

  // ESP32 PWM
  ledcSetup(channelLED, 2000, 8);
  ledcAttachPin(LED, channelLED);
  ledcWrite(channelLED, 0);
}

void loop()
{
  ledcWrite(channelLED, 0);
  delay(500);
  ledcWrite(channelLED, 255);
  delay(500);
}