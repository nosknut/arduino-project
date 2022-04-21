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
    // void update(Button &button, unsigned long maxPressDurationMs)
    // Use the version below if this code is running in tinkercad
    // https://stackoverflow.com/questions/66213706/tinkercad-function-return-user-defined-class-does-not-name-type-in-arduino
    void update(class Button &button, unsigned long maxPressDurationMs)
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
    // void update(Button &button, unsigned long minPressDurationMs)
    // Use the version below if this code is running in tinkercad
    // https://stackoverflow.com/questions/66213706/tinkercad-function-return-user-defined-class-does-not-name-type-in-arduino
    void update(class Button &button, unsigned long minPressDurationMs)
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

class BlinkController
{
private:
    Timer timer;
    unsigned int timesToBlink = 0;
    bool state = false;

public:
    void reset()
    {
        timesToBlink = 0;
        state = false;
        timer.reset();
    }

    void setBlinks(unsigned int numBlinks)
    {
        reset();
        timesToBlink = numBlinks;
        timer.loopWaitShouldReturnTrueNextTime();
    }

    /**
     * @param ledPin pin of the led to blink (remember pinMode in setup())
     * @param interval time in ms per blink
     */
    void update(int ledPin, unsigned long interval)
    {
        if ((timesToBlink > 0) && timer.loopWait(interval))
        {
            if (state)
            {
                timesToBlink--;
            }

            state = !state;
            digitalWrite(ledPin, state);
        }
    }
};

Button button(2, true);
ChangeDetector changeDetector = ChangeDetector(false);
int greenLedPin = 3;
BlinkController blinkController;
int blinkIntervalMs = 500;

void setup()
{
    Serial.begin(9600);
    button.setupButton();
    pinMode(greenLedPin, OUTPUT);
}

/**
 * Stimm-Leroy
 * Solution without classes:
 * https://www.tinkercad.com/things/1sJzcrhlt5x
 */
void loop()
{
    button.update();
    blinkController.update(greenLedPin, blinkIntervalMs);

    changeDetector.update(button.hasBeenReleasedFor(10000));
    if (changeDetector.changedToTrue())
    {
        blinkController.setBlinks(3);
    }
}
