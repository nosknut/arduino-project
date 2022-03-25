#include <Arduino.h>

#include "AlarmState.h"
#include <OledState.h>

// https://github.com/RobTillaart/RunningMedian/blob/master/examples/RunningMedian/RunningMedian.ino
#include <RunningMedian.h>

// https://github.com/Martinsos/arduino-lib-hc-sr04
#include <HCSR04.h>

// https://github.com/bxparks/AceButton
#include <AceButton.h>
using namespace ace_button;

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

struct PinConfig
{
    const byte light = A0;
    const byte temp = A1;
    const byte button = 6;
    const byte sonic_echo = 7;
    const byte sonic_trigger = 8;
    const byte buzzer = 9;
} const pinConfig;

struct ApplicationConfig
{
    const int alarmDistanceCmTreshold = 5;
    const int medianFilterWindowSize = 5;
    const float referenceVoltage = 5.0;

    // OLED Display
    const int screenWidth = 128; // OLED display width, in pixels
    const int screenHeight = 64; // OLED display height, in pixels

    // Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
    // The pins for I2C are defined by the Wire-library.
    // On an arduino UNO:       A4(SDA), A5(SCL)
    // On an arduino MEGA 2560: 20(SDA), 21(SCL)
    // On an arduino LEONARDO:   2(SDA),  3(SCL), ...
    const byte oledReset = -1;       // Reset pin # (or -1 if sharing Arduino reset pin)
    const byte screenAddress = 0x3C; ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
} const appConfig;

// Median filters to remove spikes from sensor readings
struct Filters
{
    // Keep the window filter size small to avoid slow response times
    RunningMedian temp = RunningMedian(appConfig.medianFilterWindowSize);
    RunningMedian distance = RunningMedian(appConfig.medianFilterWindowSize);
    RunningMedian light = RunningMedian(appConfig.medianFilterWindowSize);
};

struct ApplicationState
{
    OledState oled = OledState::DISTANCE;
    AlarmState alarm = AlarmState::OFF;
    Filters filters;
} state;

// https://github.com/Martinsos/arduino-lib-hc-sr04
UltraSonicDistanceSensor distanceSensor = UltraSonicDistanceSensor(pinConfig.sonic_trigger, pinConfig.sonic_echo);
// Initialize in void setup()
AceButton button(pinConfig.button, HIGH);
// OLED Display Setup
Adafruit_SSD1306 display(appConfig.screenWidth, appConfig.screenHeight, &Wire, appConfig.oledReset);

void onClick(AceButton *button, uint8_t eventType, uint8_t buttonState)
{
    if (eventType == AceButton::kEventPressed)
    {
        if (state.alarm == AlarmState::ON)
        {
            state.alarm = AlarmState::OVERRIDE;
        }
        else
        {
            state.oled = getNextOledState(state.oled);
        }
    }
}

void setupDisplay()
{
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if (!display.begin(SSD1306_SWITCHCAPVCC, appConfig.screenAddress))
    {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ; // Don't proceed, loop forever
    }
}

void setup()
{
    Serial.begin(9600);
    pinMode(pinConfig.light, INPUT);
    pinMode(button.getPin(), INPUT_PULLUP);
    button.setEventHandler(onClick);
    setupDisplay();
}

double getVoltageFor(int pin)
{
    int value = analogRead(pin);
    return (value * appConfig.referenceVoltage) / 1023;
}

double getTemperatureCelsiusFor(int pin)
{
    const float voltageAtZeroDeg = 0.5;
    const int tempScaleFactor = 100;
    const auto voltage = getVoltageFor(pin);
    return (voltage - voltageAtZeroDeg) * tempScaleFactor;
}

void updateAlarmState()
{
    const auto distanceCm = state.filters.distance.getMedian();
    // Activate alarm if within treshold
    if (distanceCm < appConfig.alarmDistanceCmTreshold)
    {
        // Unless manually overridden
        if (state.alarm != AlarmState::OVERRIDE)
        {
            state.alarm = AlarmState::ON;
        }
    }
    else
    {
        // And remove the ON/OVERRIDE when no longer within the treshold
        state.alarm = AlarmState::OFF;
    }
}

void updateFilters()
{
    state.filters.light.add(getVoltageFor(pinConfig.light));
    // Must run before distance.add() to provide initial temp value
    state.filters.temp.add(getTemperatureCelsiusFor(pinConfig.temp));
    state.filters.distance.add(distanceSensor.measureDistanceCm(state.filters.temp.getMedian()));
}

void printSensorValue(String label, float value, String unit)
{
    display.print(label);
    display.print(": ");
    display.print(value);
    display.print(" ");
    display.println(unit);
}

void updateDisplay()
{

    const auto photoresistorVoltage = state.filters.light.getMedian();
    const auto distanceCm = state.filters.distance.getMedian();
    const auto tempCelsius = state.filters.temp.getMedian();

    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);

    if (state.alarm == AlarmState::ON)
    {
        display.println("ALARM");
    }
    else
    {
        switch (state.oled)
        {
        case OledState::DISTANCE:
            printSensorValue("Distance", distanceCm, "cm");
            break;
        case OledState::TEMPERATURE:
            printSensorValue("Temp", tempCelsius, "C");
            break;
        case OledState::LIGHT:
            printSensorValue("Light", photoresistorVoltage, "V");
            break;
        }
    }
    display.display();
}

void updateBuzzer()
{
    if (state.alarm == AlarmState::ON)
    {
        tone(pinConfig.buzzer, 1000);
    }
    else
    {
        noTone(pinConfig.buzzer);
    }
}

void loop()
{
    delay(10);
    button.check();
    updateFilters();
    updateAlarmState();
    updateDisplay();
    updateBuzzer();
}
