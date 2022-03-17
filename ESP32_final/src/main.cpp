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
  BUTTON,
  LED_ROW
};

OppgaveStatus oppgave = OppgaveStatus::OPPGAVE2_A;

OppgaveStatus getNextStatus(OppgaveStatus currentState)
{
  switch (currentState)
  {
  case OppgaveStatus::OPPGAVE2_A:
    return OppgaveStatus::OPPGAVE2_B;
  case OppgaveStatus::OPPGAVE2_B:
    return OppgaveStatus::OPPGAVE3_B;
  case OppgaveStatus::OPPGAVE3_B:
    return OppgaveStatus::TEST;
  case OppgaveStatus::TEST:
    return OppgaveStatus::OLED;
  case OppgaveStatus::OLED:
    return OppgaveStatus::BUTTON;
  case OppgaveStatus::BUTTON:
    return OppgaveStatus::LED_ROW;
  case OppgaveStatus::LED_ROW:
    return OppgaveStatus::OPPGAVE2_A;
  }
}

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

// led - row
const int BLUE_LED = 4; // D4 - pin
const int blueLED_channel = 4;
const int GREEN_LED = 16; // RX2 - pin
const int greenLED_channel = 2;
const int RED_LED = 17; // TX2 - pin
const int redLED_channel = 0;
const int YELLOW_LED = 5; // TX2 - pin
const int yellowLED_channel = 6;

const int LED = 32;

// channels
const int channelLED = 0;

void setupLED(int ledPin, int channelLED)
{
  pinMode(ledPin, OUTPUT);
  ledcSetup(channelLED, 2000, 8);
  ledcAttachPin(ledPin, channelLED);
  ledcWrite(channelLED, 0);
}

void setup()
{
  Serial.begin(9600);
  // led - row
  setupLED(BLUE_LED, blueLED_channel);
  setupLED(GREEN_LED, greenLED_channel);
  setupLED(RED_LED, redLED_channel);
  setupLED(YELLOW_LED, yellowLED_channel);

  setupLED(LED, channelLED);

  buttonOne.setDebounceTime(DEBOUNCE_TIME);

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
  buttonOne.loop();
  if (buttonOne.isPressed())
  {
    oppgave = getNextStatus(oppgave);
  }
  if (oppgave == OppgaveStatus::OPPGAVE2_A)
  {
    ledcWrite(channelLED, 0);
    display.clearDisplay();
    display.setTextSize(1.5);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.print("Oppgave2 A: ");
    display.display();
    delay(1000);

    display.clearDisplay();
    display.display();
    ledcWrite(channelLED, 255);
    delay(1000);
  }
  if (oppgave == OppgaveStatus::OPPGAVE2_B)
  {
    Serial.println("Hello World!");

    display.clearDisplay();
    display.setTextSize(1.5);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.print("Oppgave2 B: ");
    display.setCursor(0, 10);
    display.print("HELLO WORLD!");
    display.display();
  }
  if (oppgave == OppgaveStatus::TEST)
  {
    buttonOne.loop();

    int analogValue = analogRead(POT_PIN);
    Serial.println(analogValue);
    int ledStrength = map(analogValue, 0, 4096, 0, 255);
    ledcWrite(channelLED, ledStrength);

    display.clearDisplay();
    display.setTextSize(1.5);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.print("TEST: ");
    display.setCursor(0, 10);
    display.print("Skru pot meteret:)");
    display.display();
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
  if (oppgave == OppgaveStatus::LED_ROW)
  {
    display.clearDisplay();
    display.setTextSize(1.5);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.print("LED_ROW: ");
    display.setCursor(0, 10);
    display.print("Skru pot meteret:)");
    display.display();
    int analogValue = analogRead(POT_PIN);
    int runningMedianPot = map(analogValue, 0, 4095, 0, 10000);
    // led_row
    // YELLOE
    if (runningMedianPot < 2500)
    {
      int yellow = map(runningMedianPot, 0, 2500, 0, 255);
      ledcWrite(yellowLED_channel, yellow);
      ledcWrite(redLED_channel, 0);
      ledcWrite(greenLED_channel, 0);
      ledcWrite(blueLED_channel, 0);
    }
    if (runningMedianPot >= 2500 && runningMedianPot <= 5000)
    {
      int yellow = map(runningMedianPot, 2500, 5000, 255, 0);
      ledcWrite(yellowLED_channel, yellow);
      int red = map(runningMedianPot, 2500, 5000, 0, 255);
      ledcWrite(redLED_channel, red);
      ledcWrite(greenLED_channel, 0);
      ledcWrite(blueLED_channel, 0);
    } // RED
    if (runningMedianPot > 5000 && runningMedianPot <= 7500)
    {
      int green = map(runningMedianPot, 5000, 7500, 0, 255);
      ledcWrite(greenLED_channel, green);
      int red = map(runningMedianPot, 5000, 7500, 255, 0);
      ledcWrite(redLED_channel, red);
      ledcWrite(yellowLED_channel, 0);
      ledcWrite(blueLED_channel, 0);
    }
    if (runningMedianPot > 7500)
    {
      int green = map(runningMedianPot, 7500, 10000, 255, 0);
      ledcWrite(greenLED_channel, green);
      int blue = map(runningMedianPot, 7500, 10000, 0, 255);
      ledcWrite(blueLED_channel, blue);
      ledcWrite(yellowLED_channel, 0);
      ledcWrite(redLED_channel, 0);
    }
  }
}