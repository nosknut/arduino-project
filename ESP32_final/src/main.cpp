#include <Arduino.h>
#include <ezButton.h>
// lagt til lib_deps = ezButton... tror d er riktig

// inneholder A til B
enum class OppgaveStatus
{
  OPPGAVE2_A,
  OPPGAVE2_B,
  OPPGAVE3_A,
  OPPGAVE3_B,
  TEST
};

//-----VELG HVILKEN OPPGVAVE SOM SKAL KJØRES------
OppgaveStatus oppgave = OppgaveStatus::TEST;

// ledcSetup(PWM - kanal(0 - 15), PWM.frekvens, PWM - oppløsning(i bit));

// ezButton buttonLED = 33;
#define BUTTON_PIN 33    // port fra knapp til ESP32
#define DEBOUNCE_TIME 50 // debounce tid (millisekunder)
// Variables will change:
int lastSteadyState = LOW;      // the previous steady state from the input pin
int lastFlickerableState = LOW; // the previous flickerable state from the input pin
int currentState;               // the current reading from the input pin

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0; // the last time the output pin was toggled

const int LED = 32;

// channels
const int channelLED = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

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
    // read the state of the switch/button:
    currentState = digitalRead(BUTTON_PIN);

    // check to see if you just pressed the button
    // (i.e. the input went from LOW to HIGH), and you've waited long enough
    // since the last press to ignore any noise:

    // If the switch/button changed, due to noise or pressing:
    if (currentState != lastFlickerableState)
    {
      // reset the debouncing timer
      lastDebounceTime = millis();
      // save the the last flickerable state
      lastFlickerableState = currentState;
    }

    if ((millis() - lastDebounceTime) > DEBOUNCE_TIME)
    {
      // whatever the reading is at, it's been there for longer than the debounce
      // delay, so take it as the actual current state:

      // if the button state has changed:
      if (lastSteadyState == HIGH && currentState == LOW)
      {
        Serial.println("The button is pressed");
        ledcWrite(channelLED, 255);
      }
      else if (lastSteadyState == LOW && currentState == HIGH)
      {
        Serial.println("The button is released");
        ledcWrite(channelLED, 0);
      }

      // save the the last steady state
      lastSteadyState = currentState;
    }
  }
}