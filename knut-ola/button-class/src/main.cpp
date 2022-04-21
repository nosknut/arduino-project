#include <Arduino.h>

class Timer
{
private:
    unsigned long startTime;
    unsigned long endTime;
    // Allows the timer to return true the first run of a loop
    // For example if you want to print something every 2 seconds,
    // calling this will print once immedietly and then every 2 seconds
    bool loopWaitHasReturnedTrueOnce = true;

public:
    Timer()
    {
        reset();
    }

    /**
     * @brief Will reset the timer
     *
     */
    void reset()
    {
        startTime = millis();
        endTime = 0;
    }

    /**
     * @brief Will reset the timer
     *
     */
    void stop()
    {
        endTime = millis();
    }

    /**
     * @brief Get the time in milliseconds since the timer was started or reset
     *
     */
    unsigned long getElapsedTime()
    {
        if (endTime == 0)
        {
            return millis() - startTime;
        }
        else
        {
            return endTime - startTime;
        }
    }

    /**
     *  Will check if the time since last reset()
     *  call is greater than the given time
     *  Note that while loopWait() automatically resets after
     *  the given time, this function does not
     */
    bool isFinished(const unsigned long durationMs)
    {
        return getElapsedTime() >= durationMs;
    }

    // Allows the timer to return true the first run of a loop
    // For example if you want to print something every 2 seconds,
    // calling this will print once immedietly and then every 2 seconds
    void loopWaitShouldReturnTrueNextTime()
    {
        loopWaitHasReturnedTrueOnce = false;
    }

    /**
     * Will return false until the given time has passed
     * Then it will return true and start counting down the same amount again
     * \code{.cpp}
     * // Example
     * Timer timer;
     * while(true) {
     *     if(timer.loopWait(1000)) {
     *         // Will run every 1000ms
     *     }
     * }
     * \endcode
     */
    bool loopWait(const unsigned long durationMs)
    {
        // Allows the timer to return true the first run of a loop
        // For example if you want to print something every 2 seconds,
        // calling this will print once immedietly and then every 2 seconds
        if (!loopWaitHasReturnedTrueOnce)
        {
            loopWaitHasReturnedTrueOnce = true;
            return true;
        }
        if (isFinished(durationMs))
        {
            reset();
            return true;
        }
        return false;
    }
};

class ChangeDetector
{

private:
    bool value;
    bool changedThisCycle = false;

public:
    ChangeDetector(bool initialValue) : value(initialValue)
    {
    }

    void update(bool currentValue)
    {
        if (currentValue != value)
        {
            value = currentValue;
            changedThisCycle = true;
        }
        else
        {
            changedThisCycle = false;
        }
    }

    bool detectedChange()
    {
        return changedThisCycle;
    }

    bool changedToTrue()
    {
        return (value == true) && detectedChange();
    }

    bool changedToFalse()
    {
        return (value == false) && detectedChange();
    }

    bool getCurrentValue()
    {
        return value;
    }

    bool getPreviousValue()
    {
        if (detectedChange())
        {
            // If there was a change then the previous value
            // is the opposite of the current value
            return !value;
        }
        else
        {
            return value;
        }
    }
};

class Button
{
    // private means that all variables and functions
    // below this line will NOT be accessible outside
    // of this class
private:
    // Configs
    // (const means that the variable can not be changed after it has been set)
    const int buttonPin;
    const bool isPullup;

    // Button states
    ChangeDetector pressChangeDetector = ChangeDetector(false);
    // Button is released when the arduino starts
    ChangeDetector releaseChangeDetector = ChangeDetector(true);

    // Timers
    Timer durationOfCurrentPressTimer;
    Timer timeSinceLastPressTimer;
    Timer timeSinceLastReleaseTimer;

    // Click counters
    unsigned int timesPressed = 0;
    unsigned int timesReleased = 0;

    bool readButtonValue()
    {
        if (isPullup)
        {
            return digitalRead(buttonPin) == LOW;
        }
        else
        {
            return digitalRead(buttonPin) == HIGH;
        }
    }

    // public means that all variables and functions
    // below this line WILL be accessible outside of
    // this class
public:
    // This is the constructor of the class
    // It is called when you create a new button
    // Use it like this:
    // Button button(5, true);
    // This will create a button you can use in your code,
    // that will listen to pin 5, and will assume you have used
    // a pullup resistor in the button circuit
    Button(int pin, bool pullup = true)
        : buttonPin(pin),
          isPullup(pullup)
    {
        // We should not start counting down the
        // release timer until the button is pressed
        // for the first time
        timeSinceLastReleaseTimer.stop();
    }

    // Call this function once, and only once, per loop
    // If possible, call it at the top of your loop
    void update()
    {
        bool buttonValue = readButtonValue();

        pressChangeDetector.update(buttonValue);
        releaseChangeDetector.update(!buttonValue);

        if (pressChangeDetector.changedToTrue())
        {
            timesPressed++;
            timeSinceLastPressTimer.reset();
            durationOfCurrentPressTimer.reset();
        }

        if (releaseChangeDetector.changedToTrue())
        {
            timesReleased++;
            timeSinceLastReleaseTimer.reset();
            durationOfCurrentPressTimer.stop();
        }
    }

    bool isPressed()
    {
        return pressChangeDetector.getCurrentValue();
    }

    bool wasPressedThisCycle()
    {
        return pressChangeDetector.changedToTrue();
    }

    bool wasReleasedThisCycle()
    {
        return releaseChangeDetector.changedToTrue();
    }

    /**
     * @return the duration of the most recent press
     * The timer stops counting after the button is
     * released and starts counting again when the button
     * is pressed again
     */
    unsigned long getTimePressed()
    {
        return durationOfCurrentPressTimer.getElapsedTime();
    }

    /**
     * @return the duration of the most recent release
     * The timer returns 0 (zero) after the button is
     * pressed and starts counting again when the button
     * is released again
     */
    unsigned long getTimeReleased()
    {
        if (isPressed())
        {
            return 0;
        }
        return timeSinceLastReleaseTimer.getElapsedTime();
    }

    unsigned int getTimesPressed()
    {
        return timesPressed;
    }

    unsigned int getTimesReleased()
    {
        return timesReleased;
    }

    void resetTimesPressed()
    {
        timesPressed = 0;
    }

    void resetTimesReleased()
    {
        timesReleased = 0;
    }

    void resetClickCounters()
    {
        resetTimesPressed();
        resetTimesReleased();
    }

    void resetTimers()
    {
        timeSinceLastPressTimer.reset();
        timeSinceLastReleaseTimer.reset();
        durationOfCurrentPressTimer.reset();
    }

    void reset()
    {
        resetClickCounters();
        resetTimers();
    }

    /**
     * @param durationMs how long the button must be pressed for the function to return true
     * @return true If the button was pressed for the given duration
     */
    bool hasBeenPressedFor(const unsigned long durationMs)
    {
        if (isPressed())
        {
            return getTimePressed() >= durationMs;
        }
        else
        {
            return false;
        }
    }

    /**
     * @param durationMs how long the button must be released for the function to return true
     * @return true If the button was released for the given duration
     */
    bool hasBeenReleasedFor(const unsigned long durationMs)
    {
        if (isPressed())
        {
            return false;
        }
        else
        {
            return getTimeReleased() >= durationMs;
        }
    }

    bool wasReleasedWithin(const unsigned long durationMs)
    {
        if (isPressed())
        {
            return false;
        }
        else
        {
            // The timer stops counting after the button is released
            return getTimePressed() <= durationMs;
        }
    }

    void setupButton()
    {
        pinMode(buttonPin, INPUT);
    }
};

class ShortPressCounter
{
private:
    // button.wasReleasedWithin() will be true when the microcontroller starts
    ChangeDetector changeDetector = ChangeDetector(true);
    unsigned int timesShortPressed = 0;

public:
    void reset()
    {
        timesShortPressed = 0;
    }

    bool countedUpThisCycle()
    {
        return changeDetector.changedToTrue();
    }

    unsigned int getCount()
    {
        return timesShortPressed;
    }

    bool atLeast(unsigned int times)
    {
        return timesShortPressed >= times;
    }

    bool atMost(unsigned int times)
    {
        return timesShortPressed <= times;
    }

    bool exactly(unsigned int times)
    {
        return timesShortPressed == times;
    }

    /**
     * @param numberOfPresses how many presses between each time the function returns true
     * @return true every time the button is pressed numberOfPresses times
     */
    bool every(unsigned int numberOfPresses)
    {
        if (timesShortPressed == 0)
        {
            return false;
        }
        return (timesShortPressed % numberOfPresses) == 0;
    }

    /**
     * @brief Used to keep track of short presses.
     *
     * @param maxPressDurationMs the maximum duration of a press.
     * If this duration is exceeded, the counter will be reset
     */
    void update(Button &button, unsigned long maxPressDurationMs)
    // Use the version below if this code is running in tinkercad
    // https://stackoverflow.com/questions/66213706/tinkercad-function-return-user-defined-class-does-not-name-type-in-arduino
    // void update(class Button &button, unsigned long maxPressDurationMs)
    {
        if (button.hasBeenPressedFor(maxPressDurationMs))
        {
            // Reset the click counter if the button has been pressed for too long
            reset();
        }

        // We use the change detector to make sure we only count clicks one time
        changeDetector.update(button.wasReleasedWithin(maxPressDurationMs));
        if (changeDetector.changedToTrue())
        {
            timesShortPressed++;
        }
    }
};

class LongPressCounter
{
private:
    ChangeDetector changeDetector = ChangeDetector(false);
    unsigned int timesLongPressed = 0;

public:
    void reset()
    {
        timesLongPressed = 0;
    }

    bool countedUpThisCycle()
    {
        return changeDetector.changedToTrue();
    }

    unsigned int getCount()
    {
        return timesLongPressed;
    }

    bool atLeast(unsigned int times)
    {
        return timesLongPressed >= times;
    }

    bool atMost(unsigned int times)
    {
        return timesLongPressed <= times;
    }

    bool exactly(unsigned int times)
    {
        return timesLongPressed == times;
    }

    /**
     * @param numberOfPresses how many presses between each time the function returns true
     * @return true every time the button is pressed numberOfPresses times
     */
    bool every(unsigned int numberOfPresses)
    {
        if (timesLongPressed == 0)
        {
            return false;
        }
        return (timesLongPressed % numberOfPresses) == 0;
    }

    /**
     *
     * @brief Used to detect a specific number of long presses
     *
     * @param minPressDurationMs the minimum duration of a press.
     * If this duration is not exceeded, the counter will be reset
     */
    void update(Button &button, unsigned long minPressDurationMs)
    // Use the version below if this code is running in tinkercad
    // https://stackoverflow.com/questions/66213706/tinkercad-function-return-user-defined-class-does-not-name-type-in-arduino
    // void update(class Button &button, unsigned long minPressDurationMs)
    {
        // We use the change detector to make sure we only count clicks one time
        changeDetector.update(button.hasBeenPressedFor(minPressDurationMs));
        if (changeDetector.changedToTrue())
        {
            timesLongPressed++;
        }

        if (button.wasReleasedWithin(minPressDurationMs))
        {
            // If the button was released within the
            // minimum duration, it was not a long press
            reset();
        }
    }
};

Button button(12, true);
ShortPressCounter shortPress500;
LongPressCounter longPress1000;
LongPressCounter longPress2000;

ChangeDetector detectPressFor1000(false);
ChangeDetector detectPressFor2000(false);
ChangeDetector detectPressFor4000(false);
ChangeDetector detectReleaseFor2000(false);
ChangeDetector detectReleaseFor4000(false);

void setup()
{
    Serial.begin(9600);
    button.setupButton();
    pinMode(LED_BUILTIN, OUTPUT);
}

void printLowLevelButtonState()
{
    Serial.print(button.isPressed());
    Serial.print(", ");
    Serial.print(button.wasPressedThisCycle());
    Serial.print(", ");
    Serial.print(button.wasReleasedThisCycle());
    Serial.print(", ");
    Serial.print(button.getTimesPressed());
    Serial.print(", ");
    Serial.print(button.getTimesReleased());
    Serial.print(", ");
    Serial.print(button.getTimePressed());
    Serial.print(", ");
    Serial.print(button.getTimeReleased());

    Serial.println("");
}

void printBasicLogic()
{
    Serial.print(button.hasBeenPressedFor(1000));
    Serial.print(", ");
    Serial.print(button.hasBeenPressedFor(2000));
    Serial.print(", ");
    Serial.print(button.hasBeenReleasedFor(1000));
    Serial.print(", ");
    Serial.print(button.hasBeenReleasedFor(2000));
    Serial.print(", ");
    Serial.print(button.wasReleasedWithin(500));
    Serial.print(", ");
    Serial.print(button.wasReleasedWithin(1000));
    Serial.print(", ");

    Serial.println("");
}

void printAtShortAndLongPressLogic()
{
    longPress1000.update(button, 1000);
    longPress2000.update(button, 2000);
    shortPress500.update(button, 500);

    Serial.print(shortPress500.atLeast(3));
    Serial.print(", ");
    Serial.print(shortPress500.exactly(3));
    Serial.print(", ");
    Serial.print(shortPress500.every(3));
    Serial.print(", ");
    Serial.print(shortPress500.getCount());
    Serial.print(" --- ");
    Serial.print(longPress1000.atLeast(3));
    Serial.print(", ");
    Serial.print(longPress1000.exactly(3));
    Serial.print(", ");
    Serial.print(longPress1000.every(3));
    Serial.print(", ");
    Serial.print(longPress1000.getCount());
    Serial.print(", ");
    Serial.print(" --- ");
    Serial.print(longPress2000.atLeast(3));
    Serial.print(", ");
    Serial.print(longPress2000.exactly(3));
    Serial.print(", ");
    Serial.print(longPress2000.every(3));
    Serial.print(", ");
    Serial.print(longPress2000.getCount());
    Serial.print(", ");

    Serial.println("");
}

void printRunOneTimeLogic()
{
    detectPressFor1000.update(button.hasBeenPressedFor(1000));
    detectPressFor2000.update(button.hasBeenPressedFor(2000));
    detectPressFor4000.update(button.hasBeenPressedFor(4000));
    detectReleaseFor2000.update(button.hasBeenReleasedFor(2000));
    detectReleaseFor4000.update(button.hasBeenReleasedFor(4000));

    Serial.print(detectPressFor1000.changedToTrue());
    Serial.print(", ");
    // Calling it multiple times will not change the return value
    Serial.print(detectPressFor1000.changedToTrue());
    Serial.print(", ");
    Serial.print(detectPressFor2000.changedToTrue());
    Serial.print(", ");
    Serial.print(detectPressFor4000.changedToTrue());
    Serial.print(" --- ");
    Serial.print(detectReleaseFor2000.changedToTrue());
    Serial.print(", ");
    Serial.print(detectReleaseFor4000.changedToTrue());

    Serial.println("");
}

void loop()
{
    button.update();

    detectPressFor1000.update(button.hasBeenPressedFor(1000));
    detectReleaseFor2000.update(button.hasBeenReleasedFor(2000));
    longPress2000.update(button, 2000);
    shortPress500.update(button, 500);

    if (button.wasPressedThisCycle())
    {
        // Will run once when the button is pressed down
        Serial.println("Button was pressed");
    }

    if (button.wasReleasedThisCycle())
    {
        // Will run once when the button is reeased
        Serial.print("Button was released after ");
        Serial.print(button.getTimePressed());
        Serial.println(" milliseconds");
    }

    if (detectPressFor1000.changedToTrue())
    {
        // Will run one time 1 second after the button is pressed
        Serial.println("Button was pressed for 1 second");
    }

    if (button.hasBeenPressedFor(2000))
    {
        // Will run constantly after a 2 second delay
        // Uncomment the next line if you want to see
        // Serial.println("Button was pressed for at least 2 seconds");
        digitalWrite(LED_BUILTIN, HIGH);
    }

    if (button.hasBeenReleasedFor(2000))
    {
        // Will run constantly after a 2 second delay
        // Uncomment the next line if you want to see
        // Serial.println("Button was released at least 2 seconds ago");
        digitalWrite(LED_BUILTIN, LOW);
    }

    if (detectReleaseFor2000.changedToTrue())
    {
        // Will run one time 2 seconds after the button is released
        Serial.println("Button was released 2 seconds ago");
    }

    if (longPress2000.exactly(2) && longPress2000.countedUpThisCycle())
    {
        // Will run one time after a 2 long presses that lasted for 2 seconds
        Serial.println("Button was long pressed 2 times for 2 seconds");
    }

    if (longPress2000.exactly(3) && longPress2000.countedUpThisCycle())
    {
        // Will run one time after a 3 long presses that lasted for 2 seconds
        Serial.println("Button was long pressed 3 times for 2 seconds");
    }

    if (longPress2000.atLeast(4))
    {
        // We did not use countedUpThisCycle here
        // If the button is long pressed 4 times for 2 seconds,
        // this will run constantly until the button is
        // pressed again, or you call button.resetClickCounters()
    }

    if (shortPress500.exactly(3))
    {
        // It is also possible to reset the click counter manually
        shortPress500.reset();
        Serial.println("Button was short pressed 3 times");

        Timer maxTimer;
        Timer timer;
        // The next line will cause the if-statement inside the
        // loop to print "Waiting 2 seconds" immedietly, and then wait
        // 2 seconds before printing it again
        timer.loopWaitShouldReturnTrueNextTime();

        // Exit this while loop after 10 seconds
        while (!maxTimer.isFinished(10000))
        {
            button.update();
            longPress2000.update(button, 2000);

            // Repeat every 2 seconds
            if (timer.loopWait(2000))
            {
                Serial.println("Waiting 2 seconds");
                Serial.println("Long press the button to abort");
            }

            // If the button is held down for 2 seconds
            if (longPress2000.countedUpThisCycle())
            {
                Serial.println("Button was long pressed 1 time for 2 seconds");
                Serial.println("The button has been pressed " + String(button.getTimesPressed()) + " times so far");
                button.resetClickCounters();
                Serial.println("Aborting the loop ...");
                // Exit the while-loop
                break;
            }
        }
    }
}
