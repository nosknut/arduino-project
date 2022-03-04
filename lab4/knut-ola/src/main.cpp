#include <Arduino.h>
#include <Zumo32U4.h>
#include <Timer.h>
#include <Range.h>
#include <Sequence.h>
#include <PidController.h>

// Sequences
Sequence exercise1Sequence;
Sequence moveEightSequence;
Sequence exercise2Sequence;
Sequence flashAllLedsSequence;
Sequence calibrationSequence;
Sequence printPositionSequence;
Sequence followLineSequence;
Sequence exercise3Sequence;
Sequence exercise4Sequence;
Sequence loopSequence;
Sequence lcdScrollSequence;

const int maxSpeed = 150;
const Range inputRange(0, 4000);
const Range outputRange(0, 2 * maxSpeed);
PidController linePidController(10.0, 0.0, 0.0, inputRange, outputRange);

Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4LCD lcd;

const int lcdWidth = 8;
const int NUM_SENSORS = 5;
unsigned int lineSensorValues[NUM_SENSORS];

void setup()
{
    Serial.begin(9600);
    lineSensors.initFiveSensors();
}

void exercise1()
{
    // By adding  || exercise1Sequence.hasFinished()
    // we ensure that the sequence will execute in its entirety
    // even after the button is released. This ensures that
    // the led will have the same state as it appears
    // at the end of the sequence.
    if (buttonA.isPressed() || exercise1Sequence.hasFinished())
    {
        exercise1Sequence
            .then([&] { //
                ledGreen(true);
            })
            .delay(100)
            .then([&] { //
                ledGreen(false);
            })
            .delay(100)
            .loop()
            .endOfSequence();
    }
}

/**
 * @return true when sequence is finished
 */
bool moveEight(int speed)
{
    return moveEightSequence
        .then([&] { //
            motors.setSpeeds(0, speed);
        })
        .delay(2000)
        .then([&] { //
            motors.setSpeeds(speed, speed);
        })
        .delay(1000)
        .then([&] { //
            motors.setSpeeds(speed, 0);
        })
        .delay(2000)
        .loop()
        .endOfSequence();
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
    exercise2Sequence
        .thenWhenReturnsTrue([&] { //
            // moveEight returns true when its sequence has finished
            // this function will continue to the next step when
            // its return value is true.
            // That means that this sequence will continue
            // after the moveEight sequence finishes.
            return moveEight(maxSpeed);
        })
        .thenWhenReturnsTrue([&] { //
            return moveEight(maxSpeed);
        })
        .then([&] { //
            moveCircle(maxSpeed, true);
        })
        .delay(2000)
        .then([&] { //
            moveCircle(maxSpeed, false);
        })
        .delay(2000)
        .loop()
        .endOfSequence();
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

/**
 * @return true when sequence is finished
 */
bool flashAllLeds(const int numFlashes, const int flashDuration)
{
    return flashAllLedsSequence
        .then([&] { //
            ledGreen(true);
            ledRed(true);
            ledYellow(true);
        })
        .delay(flashDuration)
        .then([&] { //
            ledGreen(false);
            ledRed(false);
            ledYellow(false);
        })
        .delay(flashDuration)
        .loopTimes(numFlashes)
        .endOfSequence();
}

/**
 * @return true when sequence is finished
 */
bool calibrate(const int speed)
{

    return calibrationSequence
        .then([&] { //
            printMessage("Starting in", "3");
        })
        .delay(1000)
        .then([&] { //
            printMessage("Starting in", "2");
        })
        .delay(1000)
        .then([&] { //
            printMessage("Starting in", "1");
        })
        .delay(1000)
        .then([&] { //
            printMessage("Calibrating", "...");
        })
        .thenWhenReturnsTrue([&] { //
            return flashAllLeds(5, 400);
        })
        .thenRunFor(2000, [&] { //
            motors.setSpeeds(-speed, speed);
            lineSensors.calibrate();
        })
        .thenRunFor(2000, [&] { //
            motors.setSpeeds(speed, -speed);
            lineSensors.calibrate();
        })
        .repeatPreviousStepsTimes(2, 2)
        .then([&] { //
            motors.setSpeeds(0, 0);
            printMessage("Done!");
        })
        .delay(1000)
        // By using loop this function can be used multiple times without
        // having to reset the sequence.
        .loop()
        .endOfSequence();
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

/**
 * @return true if the sequence is finished
 */
bool printPosition()
{
    return printPositionSequence
        .then([&] { //
            printMessage("Press A", "to cancel");
        })
        .delay(1000)
        .thenWhenReturnsTrue([&] { //
            const int position = lineSensors.readLine(lineSensorValues);
            printValue("Position: ", position);
            return buttonA.getSingleDebouncedPress();
        })
        .loop()
        .endOfSequence();
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

/**
 * @return true if the sequence is finished
 */
bool followLine()
{
    return followLineSequence
        .then([&] { //
            printMessage("Press A", "to cancel");
        })
        .delay(1000)
        .thenWhenReturnsTrue([&] { //
            printMessage("Following", "line ...");
            return flashAllLeds(10, 200);
        })
        .delay(1000)
        .thenWhenReturnsTrue([&] { //
            const int sensorValue = getLineSensorValue();

            printValue("Position: ", getProgressBar(sensorValue, inputRange));

            const int output = linePidController.update(sensorValue, 2000);
            const int centeredOutput = output - maxSpeed;

            const int leftSpeed = maxSpeed + centeredOutput;
            const int rightSpeed = maxSpeed - centeredOutput;

            motors.setSpeeds(leftSpeed, rightSpeed);

            return buttonA.getSingleDebouncedPress();
        })
        .loop()
        .endOfSequence();
}

void exercise3()
{
    exercise3Sequence
        .then([&] { //
            printMessage("Press A to", "calibrate");
        })
        .thenWhenReturnsTrue([&] { //
            return buttonA.getSingleDebouncedRelease();
        })
        .thenWhenReturnsTrue([&] { //
            return calibrate(maxSpeed);
        })
        .thenWhenReturnsTrue([&] { //
            return printPosition();
        })
        .loop()
        .endOfSequence();
}

void exercise4()
{
    exercise4Sequence
        .then([&] { //
            printMessage("Press A to", "calibrate");
        })
        .thenWhenReturnsTrue([&] { //
            return buttonA.getSingleDebouncedRelease();
        })
        .thenWhenReturnsTrue([&] { //
            return calibrate(maxSpeed);
        })
        .then([&] { //
            printMessage("Press A to", "follow line");
        })
        .thenWhenReturnsTrue([&] { //
            return buttonA.getSingleDebouncedRelease();
        })
        .thenWhenReturnsTrue([&] { //
            return followLine();
        })
        .loop()
        .endOfSequence();
}

/*
void loop1()
{
    // Keeps the LCD scrolling constantly
    // so that this does not have to be implemented
    // elsewhere in the code.
    lcdScrollSequence
        .then([&] { //
            lcd.scrollDisplayLeft();
        })
        .delay(100)
        .repeatPreviousStepsTimes(2, 20)
        .then([&] { //
            lcd.scrollDisplayRight();
        })
        .delay(100)
        .repeatPreviousStepsTimes(2, 20)
        .loop()
        .endOfSequence();

    loopSequence
        .then([&] { //
            printMessage("A: Calibrate", "B: Follow Line");
            calibrationSequence.pause();
            followLineSequence.pause();
        })
        .thenWhenReturnsTrue([&] { //
            if (buttonA.getSingleDebouncedPress())
            {
                calibrationSequence.start();
                return true;
            }
            else if (buttonB.getSingleDebouncedPress())
            {
                followLineSequence.start();
                return true;
            }
            return false;
        })
        .thenWhenReturnsTrue([&] { //
            // Only one of these sequences should be running at a time.
            // because we paused them earlier
            return calibrate(maxSpeed) || followLine();
        })
        .loop()
        .endOfSequence();
}
*/

void loop()
{
    loopSequence
        .then([&] { //
            ledGreen(true);
            Serial.println("Green on");
        })
        .delay(5000)
        .then([&] { //
            ledGreen(false);
            Serial.println("Green off");
        })
        .delay(5000)
        .loop()
        .endOfSequence();

    followLineSequence
        .then([&] { //
            ledRed(true);
            Serial.println("Red on");
        })
        .delay(100)
        .then([&] { //
            ledGreen(false);
            Serial.println("Red off");
        })
        .delay(100)
        .loop()
        .endOfSequence();
}