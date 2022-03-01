#include <Arduino.h>
#include <Zumo32U4.h>
#include <Timer.h>
#include <Range.h>

Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4LCD lcd;

const int maxSpeed = 150;
const int lcdWidth = 8;
const int NUM_SENSORS = 5;
unsigned int lineSensorValues[NUM_SENSORS];

void setup()
{
    lineSensors.initFiveSensors();
}

void exercise1()
{
    if (buttonA.isPressed())
    {
        ledGreen(true);
        delay(100);
        ledGreen(false);
        delay(100);
    }
}

void moveEight(int speed)
{
    motors.setSpeeds(0, speed);
    delay(2000);
    motors.setSpeeds(speed, speed);
    delay(1000);
    motors.setSpeeds(speed, 0);
    delay(2000);
}

void moveCircle(int speed, bool direction)
{
    int slowSpeed = speed / 3;
    if (direction)
    {
        motors.setSpeeds(slowSpeed, speed);
    }
    else
    {
        motors.setSpeeds(slowSpeed, speed);
    }
}

void exercise2()
{
    moveEight(maxSpeed);
    moveEight(maxSpeed);
    moveCircle(maxSpeed, true);
    moveCircle(maxSpeed, false);
}

void printMessage(const String message, const String message2)
{
    lcd.clear();
    lcd.gotoXY(0, 0);
    lcd.print(message);
    lcd.gotoXY(0, 1);
    lcd.print(message2);
}

void printMessage(const String message)
{
    printMessage(message, "");
}

void flashAllLeds(const int numFlashes, const int flashDuration)
{
    bool ledState = true;
    for (int i = 0; i < numFlashes; i++)
    {
        ledGreen(ledState);
        ledRed(ledState);
        ledYellow(ledState);
        ledState = !ledState;
        delay(flashDuration);
    }
}

void calibrate(const int speed)
{
    printMessage("Starting in", "3");
    delay(1000);
    printMessage("Starting in", "2");
    delay(1000);
    printMessage("Starting in", "1");
    delay(1000);
    printMessage("Calibrating", "...");
    flashAllLeds(5, 200);

    Timer timer;
    for (int i = 0; i < 3; i++)
    {
        bool direction = i % 2 == 0;
        timer.reset();
        while (!timer.isFinished(2000))
        {
            if (direction)
            {
                motors.setSpeeds(-speed, speed);
            }
            else
            {
                motors.setSpeeds(speed, -speed);
            }
            lineSensors.calibrate();
        }
    }

    motors.setSpeeds(0, 0);

    printMessage("Done!");
    delay(1000);
}

int getLineSensorValue()
{
    return lineSensors.readLine(lineSensorValues);
}

void printValue(const String message, const String value)
{
    lcd.clear();
    lcd.gotoXY(0, 0);
    lcd.print(message);
    lcd.gotoXY(0, 1);
    lcd.print(value);
}

void printValue(const String message, const int value)
{
    printValue(message, String(value));
}

void printPosition()
{
    printMessage("Press A", "to cancel");
    delay(1000);
    while (!buttonA.getSingleDebouncedPress())
    {
        const int position = lineSensors.readLine(lineSensorValues);
        printValue("Position: ", position);
        delay(10);
    }
}

int clamp(const int value, const Range range)
{
    if (value < range.minValue)
    {
        return range.minValue;
    }
    else if (value > range.maxValue)
    {
        return range.maxValue;
    }
    else
    {
        return value;
    }
}

int pid(const int value, const int targetValue, const Range inputRange, const Range outputRange)
{
    const int errorInInputScale = value - targetValue;

    const int kp = 10;

    const int outputInInputScale = kp * errorInInputScale;

    const int clampedOutputInInputScale = clamp(outputInInputScale, inputRange);

    const int outputInOutputScale = map(
        clampedOutputInInputScale,
        inputRange.minValue,
        inputRange.maxValue,
        outputRange.minValue,
        outputRange.maxValue);

    return outputInOutputScale;
}

String getProgressBar(const int value, const Range range)
{
    const int percentage = map(value, range.minValue, range.maxValue, 0, 100);
    const int numDots = map(percentage, 0, 100, 0, lcdWidth);
    String bar = "";
    for (int i = 0; i < numDots; i++)
    {
        bar += "|";
    }
    return bar;
}

void followLine()
{
    printMessage("Press A", "to cancel");
    flashAllLeds(10, 200);
    delay(1000);
    printMessage("Following", "line ...");
    delay(1000);
    while (!buttonA.getSingleDebouncedPress())
    {
        const Range inputRange(0, 4000);
        const Range outputRange(0, 2 * maxSpeed);

        const int sensorValue = getLineSensorValue();

        printValue("Position: ", getProgressBar(sensorValue, inputRange));

        const int output = pid(sensorValue, 2000, inputRange, outputRange);
        const int centeredOutput = output - maxSpeed;

        const int leftSpeed = maxSpeed + centeredOutput;
        const int rightSpeed = maxSpeed - centeredOutput;

        motors.setSpeeds(leftSpeed, rightSpeed);
    }
}

void exercise3()
{
    printMessage("Press A to", "calibrate");
    buttonA.waitForButton();
    calibrate(maxSpeed);
    printPosition();
}

void exercise4()
{
    printMessage("Press A to", "calibrate");
    buttonA.waitForButton();
    calibrate(maxSpeed);
    printMessage("Press A to", "follow line");
    buttonA.waitForButton();
    followLine();
}

void loop()
{
    printMessage("A: Calibrate", "B: Follow Line");
    while (true)
    {
        delay(400);
        lcd.scrollDisplayLeft();
        if (buttonA.getSingleDebouncedPress())
        {
            calibrate(maxSpeed);
            break;
        }
        if (buttonB.getSingleDebouncedPress())
        {
            followLine();
            break;
        }
    }
}
