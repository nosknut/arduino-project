#include <Arduino.h>
#include <kristianButton.h>
// lagt til lib_deps = ezButton... tror d er riktig

// inneholder A til B + TEST
enum class OppgaveStatus
{
  OPPGAVE2_A,
  OPPGAVE2_B,
  OPPGAVE3_A,
  OPPGAVE3_B,
  TEST
};

OppgaveStatus oppgave = OppgaveStatus::TEST;

// ledcSetup(PWM - kanal(0 - 15), PWM.frekvens, PWM - oppløsning(i bit));

#define BUTTON_PIN 33    // port fra knapp til ESP32
#define DEBOUNCE_TIME 50 // debounce tid (millisekunder)
kristianButton buttonOne = BUTTON_PIN;
// Variables will change:

const int LED = 32;

// channels
const int channelLED = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  buttonOne.debounce(50);

  // ESP32 PWM
  ledcSetup(channelLED, 2000, 8);
  ledcAttachPin(LED, channelLED);
  ledcWrite(channelLED, 0);
}

void loop()
{
  if (oppgave == OppgaveStatus::OPPGAVE2_A)
  {
    ledcWrite(channelLED, 0);
    delay(500);
    ledcWrite(channelLED, 255);
    delay(500);
  }
  if (oppgave == OppgaveStatus::OPPGAVE2_B)
  {
    Serial.println("Hello World!");
  }
  if (oppgave == OppgaveStatus::TEST)
  {
    buttonOne.setLoop();
    Serial.println(buttonOne.buttonState());
  }
}