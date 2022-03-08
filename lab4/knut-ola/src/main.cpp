#include <Arduino.h>
#include <ApplicationConfig.h>
#include <Zumo32U4.h>
#include <Timer.h>
#include <Range.h>
#include <Sequence.h>
#include <PidController.h>
#include <Scaling.h>

Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4LCD lcd;

struct Sequences
{
    // Exercises
    Sequence exercise1;
    Sequence exercise2;
    Sequence exercise3;
    Sequence exercise4;

    // Functions
    Sequence moveEight;
    Sequence flashAllLeds;
    Sequence calibration;
    Sequence printPosition;
    Sequence lcdScroll;
    Sequence loop;

    // Groups
    struct FollowLineSequences
    {
        Sequence followLine;
        Sequence printSpeed;
    } followLineGroup;
} sequences;

struct ApplicationState
{
    PidController linePidController = PidController(appConfig.linePidConfig);

    // Set by printMessage functions
    // to make the background lcd scroll
    // sequence start/stop scrolling
    bool scrollLcd = false;

    // TODO: Implement EEPROM
    // Changing the shape of this struct will change the way
    // the data is stored in EEPROM.
    // Any existing EEPROM will no longer be usable.
    struct SensorData
    {
        unsigned int lineSensorValues[APP_CONFIG_NUM_LINE_SENSORS];
    } memorizedValues;
} state;

void blockingCountdown(const String message, const int durationMillis)
{
    for (int i = durationMillis; i > 0; i--)
    {
        lcd.clear();
        lcd.gotoXY(0, 0);
        lcd.print(message);
        lcd.gotoXY(0, 1);
        lcd.print(i);
        delay(1000);
    }
    lcd.clear();
}

void setup()
{
    Serial.begin(9600);
    lineSensors.initFiveSensors();
    blockingCountdown("Starting ...", 3);
}

void exercise1()
{
    // By adding  || exercise1Sequence.hasFinished()
    // we ensure that the sequence will execute in its entirety
    // even after the button is released. This ensures that
    // the led will have the same state as it appears
    // at the end of the sequence.
    if (buttonA.isPressed() || sequences.exercise1.hasFinished())
    {
        sequences.exercise1
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
    return sequences.moveEight
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
    sequences.exercise2
        .thenWhenReturnsTrue([&] { //
            // moveEight returns true when its sequence has finished
            // this function will continue to the next step when
            // its return value is true.
            // That means that this sequence will continue
            // after the moveEight sequence finishes.
            return moveEight(appConfig.targetSpeed);
        })
        .thenWhenReturnsTrue([&] { //
            return moveEight(appConfig.targetSpeed);
        })
        .then([&] { //
            moveCircle(appConfig.targetSpeed, true);
        })
        .delay(2000)
        .then([&] { //
            moveCircle(appConfig.targetSpeed, false);
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
    state.scrollLcd = false;
}

void printValues(const float message, const float message2)
{
    printMessage(String(message, 2), String(message2, 2));
}

void printMessage(const String message)
{
    printMessage(message, "");
}

void printScrollMessage(const String message, const String message2)
{
    printMessage(message, message2);
    state.scrollLcd = true;
}

/**
 * @return true when sequence is finished
 */
bool flashAllLeds(const int numFlashes, const int flashDuration)
{
    return sequences.flashAllLeds
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
        .loopTimes(numFlashes, true)
        .endOfSequence();
}

/**
 * @return true when sequence is finished
 */
bool calibrate(const int speed)
{
    // This if will run in parallell with the sequence
    if (buttonA.getSingleDebouncedPress())
    {
        motors.setSpeeds(0, 0);
        sequences.calibration.reset();
        return true;
    }
    return sequences.calibration
        .then([&] { //
            printMessage("Press A", "to cancel");
        })
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
            return flashAllLeds(
                appConfig.numSafetyFlashesBeforeStart,
                appConfig.safetyFlashDurationMs);
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
    // NB! The variable passed to readLine is a reference that will be
    // updated by the function. Make sure you pass the ACTUAL
    // variable that should receive the changes.
    // In short, you can NOT make a variable to store the value of
    // state.memorizedValues.lineSensorValues
    // and then pass this to readLine.
    return lineSensors.readLine(state.memorizedValues.lineSensorValues);
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
    printValue(message, String(value, 2));
}

/**
 * @return true if the sequence is finished
 */
bool printPosition()
{
    return sequences.printPosition
        .then([&] { //
            printMessage("Press A", "to cancel");
        })
        .delay(3000)
        .thenWhenReturnsTrue([&] { //
            const int position = getLineSensorValue();
            printValue("Position: ", position);
            return buttonA.getSingleDebouncedPress();
        })
        .loop()
        .endOfSequence();
}

String getProgressBar(const int value, const Range range)
{
    const int numDots = Scaling::mapToRange(
        value, range, Range(0, appConfig.lcdWidth));

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
    // This if will run in parallell with the sequence
    if (buttonA.getSingleDebouncedPress())
    {
        motors.setSpeeds(0, 0);
        sequences.followLineGroup.followLine.reset();
        return true;
    }
    return sequences.followLineGroup.followLine
        .then([&] { //
            printMessage("Press A", "to cancel");
        })
        .delay(1000)
        .then([&] { //
            printMessage("Press B to", "show sp/pos");
        })
        .delay(1000)
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
            printMessage("Following", "line ...");
        })
        .thenWhenReturnsTrue([&] { //
            return flashAllLeds(
                appConfig.numSafetyFlashesBeforeStart,
                appConfig.safetyFlashDurationMs);
        })
        .thenWhenReturnsTrue([&] { //
            Range outputRange = appConfig.linePidConfig.outputRange;
            Range inputRange = appConfig.linePidConfig.inputRange;
            // const int sensorValue = currentPosition;
            const int sensorValue = getLineSensorValue();

            const int output = state.linePidController.update(sensorValue, 2000, true);

            // Assign the output to the left
            const int leftSpeed = appConfig.targetSpeed + output;
            // Take what ever is left over and assign it to the right along with the output
            const int unusedLeftSpeed = Scaling::remainderFromClamp(leftSpeed, outputRange);
            const int rightSpeed = appConfig.targetSpeed - output - unusedLeftSpeed;
            // Take what ever is left over and assign/re-assign it to the left
            const int unusedRightSpeed = Scaling::remainderFromClamp(rightSpeed, outputRange);
            const int rightCompensatedLeftSpeed = leftSpeed - unusedRightSpeed;
            // Clamp the distributed speeds to the output range
            const int clampedLeftSpeed = Scaling::clamp(rightCompensatedLeftSpeed, outputRange);
            const int clampedRightSpeed = Scaling::clamp(rightSpeed, outputRange);

            sequences.followLineGroup.printSpeed
                .thenWhenReturnsTrue([&] { //
                    printMessage(
                        "Position",
                        getProgressBar(sensorValue, inputRange));

                    return buttonB.getSingleDebouncedPress();
                })
                .thenWhenReturnsTrue([&] { //
                    printValues(sensorValue, output);
                    return buttonB.getSingleDebouncedPress();
                })
                .thenWhenReturnsTrue([&] { //
                    printValues(clampedLeftSpeed, clampedRightSpeed);
                    return buttonB.getSingleDebouncedPress();
                })
                .loop()
                .endOfSequence();

            motors.setSpeeds(clampedLeftSpeed, clampedRightSpeed);
            return buttonA.getSingleDebouncedPress();
        })
        .then([&] { //
            // Kill the motors after the user stops the program
            motors.setSpeeds(0, 0);
        })
        .loop()
        .endOfSequence();
}

void exercise3()
{
    sequences.exercise3
        .then([&] { //
            printMessage("Press A to", "calibrate");
        })
        .thenWhenReturnsTrue([&] { //
            return buttonA.getSingleDebouncedRelease();
        })
        .thenWhenReturnsTrue([&] { //
            return calibrate(appConfig.calibrationSpeed);
        })
        .thenWhenReturnsTrue([&] { //
            return printPosition();
        })
        .loop()
        .endOfSequence();
}

void exercise4()
{
    sequences.exercise4
        .then([&] { //
            printMessage("Press A to", "calibrate");
        })
        .thenWhenReturnsTrue([&] { //
            return buttonA.getSingleDebouncedRelease();
        })
        .thenWhenReturnsTrue([&] { //
            return calibrate(appConfig.calibrationSpeed);
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

// Keeps the LCD scrolling constantly
// so that this does not have to be implemented
// elsewhere in the code.
void updateLcdScroll()
{
    if (false)
    {
        sequences.lcdScroll
            .then([&] { //
                lcd.scrollDisplayLeft();
            })
            .delay(300)
            .repeatPreviousStepsTimes(2, 20)
            .then([&] { //
                lcd.scrollDisplayRight();
            })
            .repeatPreviousStepsTimes(1, 20)
            .delay(1500)
            .loop()
            .endOfSequence();
    }
}

void updateMenuControls()
{
    sequences.loop
        .then([&] { //
            printScrollMessage("A: Calibrate", "B: Follow Line");
            sequences.calibration.reset();
            sequences.followLineGroup.followLine.reset();
            sequences.calibration.pause();
            sequences.followLineGroup.followLine.pause();
        })
        .thenWhenReturnsTrue([&] { //
            if (buttonA.getSingleDebouncedPress())
            {
                sequences.calibration.start();
                return true;
            }
            else if (buttonB.getSingleDebouncedPress())
            {
                sequences.calibration.start();
                sequences.followLineGroup.followLine.start();
                return true;
            }
            return false;
        })
        // Only one of these two next sequences should be running at a time.
        // because we reset them in the steps above
        //.thenWhenReturnsTrue([&] { //
        .thenWhenReturnsTrue([&] { //
            // Will continue if the sequence is not running
            return calibrate(appConfig.calibrationSpeed);
        })
        .thenWhenReturnsTrue([&] { //
            // Will continue if the sequence is not running
            return followLine();
        })
        .loop()
        .endOfSequence();
}

void loop()
{
    updateLcdScroll();
    updateMenuControls();
    // Delay the entire loop for one millisecond
    // since our code is now blazingly fast
    delay(1);
}
