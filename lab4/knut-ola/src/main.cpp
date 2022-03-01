#include <Arduino.h>
#include <Zumo32U4.h>
#include <Timer.h>
#include <Range.h>

Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4LCD lcd;

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
    int speed = 200;
    moveEight(speed);
    moveEight(speed);
    moveCircle(speed, true);
    moveCircle(speed, false);
}

void calibrate()
{
    lcd.clear();
    lcd.gotoXY(0, 0);
    lcd.print("Press A");
    lcd.gotoXY(0, 1);
    lcd.print("to calibrate");
    buttonA.waitForButton();

    Timer timer;
    const int speed = 200;
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
    lcd.clear();
    lcd.gotoXY(0, 0);
    lcd.print("Calibration");
    lcd.gotoXY(0, 1);
    lcd.print("complete");
    delay(2000);
}

int getLineSensorValue()
{
    return lineSensors.readLine(lineSensorValues);
}

void exercise3()
{
    calibrate();

    while (!buttonA.getSingleDebouncedPress())
    {
        const int position = lineSensors.readLine(lineSensorValues);
        lcd.clear();
        lcd.gotoXY(0, 0);
        lcd.print("Position: ");
        lcd.gotoXY(0, 1);
        lcd.print(position);
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

void exercise4()
{
    calibrate();
    while (true)
    {
        const int maxSpeed = 200;
        const Range inputRange(0, 4000);
        const Range outputRange(0, 2 * maxSpeed);

        const int sensorValue = getLineSensorValue();

        const int output = pid(sensorValue, 2000, inputRange, outputRange);
        const int centeredOutput = output - maxSpeed;

        const int leftSpeed = maxSpeed + centeredOutput;
        const int rightSpeed = maxSpeed - centeredOutput;

        motors.setSpeeds(leftSpeed, rightSpeed);
    }
}

void loop()
{
    exercise3();
}
