#include <Arduino.h>
#include <ezButton.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_I2CDevice.h>
#include <RunningMedian.h>
//#include <kristianButton.h> i wish :(
// lagt til lib_deps = ezButton... tror d er riktig

// inneholder A til B + TEST
enum class OppgaveStatus
{
  OPPGAVE2_A,
  OPPGAVE2_B,
  OPPGAVE3_A,
  OPPGAVE3_B,
  TEST,
  OLED,
  BUTTON
};

OppgaveStatus oppgave = OppgaveStatus::BUTTON;

// ledcSetup(PWM - kanal(0 - 15), PWM.frekvens, PWM - oppløsning(i bit));

#define BUTTON_PIN 33 // port fra knapp til ESP32
#define POT_PIN 34
#define TEMP_PIN 35
#define PHOTO_PIN 25
#define DEBOUNCE_TIME 50 // debounce tid (millisekunder)

// OLED - skjerm
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// running median variabel
RunningMedian samples = RunningMedian(5);

float runningmedianTemp = 0.0;
float currentTempSensorReading = 0.0;

// public variabler for photoresistor
int maxPhoto = 0;
int minPhoto = 1000;
long timedOut = 1000; // valgfri hvis man ønsker dette

// ezbutton er digg
ezButton buttonOne(33);

const int LED = 32;

// channels
const int channelLED = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  buttonOne.setDebounceTime(DEBOUNCE_TIME);

  // ESP32 PWM
  ledcSetup(channelLED, 2000, 8);
  ledcAttachPin(LED, channelLED);
  ledcWrite(channelLED, 0);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }
}

float getVoltageFor(int pin)
{
  int value = analogRead(pin);
  return (value * 3.3) / 4095;
}

float getTemperatureCelsiusFor(int pin)
{
  float voltage = getVoltageFor(pin);
  return (voltage - 0.5) * 100;
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
    buttonOne.loop();

    int analogValue = analogRead(POT_PIN);
    Serial.println(analogValue);
    int ledStrength = map(analogValue, 0, 4096, 0, 255);
    ledcWrite(channelLED, ledStrength);
  }
  if (oppgave == OppgaveStatus::OLED)
  {
    int analogValue = analogRead(POT_PIN);
    buttonOne.loop();
    currentTempSensorReading = getTemperatureCelsiusFor(TEMP_PIN);
    samples.add(currentTempSensorReading);

    runningmedianTemp = samples.getMedian();

    int runningMedianPot = map(analogValue, 0, 4095, 0, 10000);
    int photoResistorReading = analogRead(PHOTO_PIN);

    // dette vil kunne gi meg veldig god map
    //  max = 1229
    if (photoResistorReading > maxPhoto)
    {
      // endres hvis den får en stor verdi
      // men resetter etter en hvis tid
      maxPhoto = photoResistorReading;
      // timedOut = millis() + 500;
    }
    // min = 300
    if (photoResistorReading < minPhoto)
    {
      // endres hvis den får en stor verdi
      // men resetter etter en hvis tid
      minPhoto = photoResistorReading;
      // timedOut = millis() + 500;
    }

    int photoToLED = map(photoResistorReading, minPhoto, maxPhoto, 0, 255);
    ledcWrite(channelLED, photoToLED);

    display.clearDisplay();
    display.setTextSize(1.5);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    // Display static text
    display.print("Pot: ");
    display.print(runningMedianPot);
    display.print(", Btn: ");
    display.println(buttonOne.getStateRaw());
    display.setCursor(0, 25);
    display.print("Temp: ");
    display.println(runningmedianTemp);
    display.setCursor(0, 45);
    display.print("Poto: ");
    display.print(photoResistorReading);
    display.print(", max: ");
    display.print(maxPhoto);
    display.print(", min: ");
    display.print(minPhoto);
    display.display();

    delay(10);
  }
  if (oppgave == OppgaveStatus::BUTTON)
  {
  }
}