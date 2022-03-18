#ifndef Sequence_h
#define Sequence_h
#include <Arduino.h>
#include <Timer.h>

/**
 *
 * Warnings:
 * The & symbol inside the capture-clause [&]
 * of the lambda will cause all variables that
 * are used in the given lambda to be referenced
 * rather than copied. Not doing this could cause
 * funky issues that are next to impossible to debug.
 *
 * In short:
 * Always start your lambdas with [&] when using Sequence!
 *
 * About the empty comments in the example below:
 * The // comments at the end of each .then({ //
 * are there to make the formatter snap codelines
 * to a new line. If you do not include this the
 * code becomes ugly and hard to read.
 *
 * Example usage:
 *
 * struct Sequences {
 *     Sequence red;
 *     Sequence yellow;
 * } sequences;
 *
 * void loop()
 * {
 *     sequences.red
 *         .then([&] { //
 *             ledRed(true);
 *         })
 *         .delay(2000)
 *         .then([&] { //
 *             ledRed(false);
 *         })
 *         .delay(2000)
 *         .loop()
 *         .endOfSequence();
 *
 *     sequences.yellow
 *         .then([&] { //
 *             ledYellow(true);
 *         })
 *         .delay(200)
 *         .then([&] { //
 *             ledYellow(false);
 *         })
 *         .delay(200)
 *         .loop()
 *         .endOfSequence();
 * }
 *
 */
class Sequence
{
private:
    int timesPreviousStepLooped = 0;

    int timesLooped = 0;
    int timesToLoop = 0;

    /**
     * @brief Keeps track of what step we should be executing
     *
     */
    int sequenceStep = 0;
    /**
     * @brief Keeps track of how many steps we have registered
     *
     */
    int checkedSteps = 0;
    /**
     * Keeps track of how many steps we registered last time
     * we ran the sequence (the last known value of checkedSteps)
     * Used to determine how many steps are in the sequence
     * and by extension, if the sequence execution has finished executing.
     *
     */
    int numSteps = 0;
    /**
     * Will be true after the last step has executed and during the execution
     * of the first step after a sequence restart.
     */
    bool sequenceHasFinished = false;
    bool shouldLoop = false;
    bool endSequenceShouldReturnTrueBetweenLoops = true;
    bool paused = false;

public:
    /**
     * Used to keep track of time in the delay() function.
     * You may use it yourself, but beware this could cause weird issues.
     *
     */
    Timer timer;
    /**
     * @brief Will move n steps back or forth in the sequence. You should usually not use this directly.
     *
     * @param deltaSteps how many steps to move back (negative) or forward (positive)
     */
    void moveSteps(int deltaSteps)
    {
        sequenceStep += deltaSteps;
        timer.reset();
    }

    /**
     * Will run the code provided and go to next sequence step
     * if the callback function returns true
     *  @param callback Function to run until it returns true
     */
    template <typename F>
    Sequence &thenWhenReturnsTrue(F callback)
    {
        if (paused)
        {
            return *this;
        }

        if (checkedSteps == sequenceStep)
        {
            if (callback())
            {
                moveSteps(1);
            }
        }
        checkedSteps += 1;
        return *this;
    }

    /**
     * @brief Will run the code provided and go to next sequence step
     *
     * @param callback a function to run before continuing to the next step
     * @return Sequence&
     */
    template <typename F>
    Sequence &then(F callback)
    {
        if (paused)
        {
            return *this;
        }
        if (checkedSteps == sequenceStep)
        {
            callback();
            moveSteps(1);
        }
        checkedSteps += 1;
        return *this;
    }

    /**
     * @brief Will reset everything and move the sequence to the first step
     *
     */
    void reset()
    {
        checkedSteps = 0;
        sequenceStep = 0;
        timesLooped = 0;
        timesPreviousStepLooped = 0;
        paused = false;
        sequenceHasFinished = false;
    }

    /**
     * @brief Will reset the sequence. If you want to resume after a pause, call resume() instead
     *
     */
    void start()
    {
        reset();
    }

    /**
     * Will pause execution the sequence, while remembering the current step.
     * If you want to start the sequence again, call resume()
     *
     */
    void pause()
    {
        paused = true;
    }

    /**
     * @brief Will resume execution of the sequence if it is paused
     *
     */
    void resume()
    {
        paused = false;
    }

    /**
     * @return int the number of times the sequence has looped.
     * This only incremens when using the loopTimes() function.
     */
    int getTimesSequenceLooped()
    {
        return timesLooped;
    }

    /**
     * @return int the number of times the repeatPreviousStepsUntil()
     * or repeatPreviousStepsTimes() functions have run. This will
     * reset when the sequence continues past those steps.
     */
    int getTimesPreviousStepLooped()
    {
        return timesPreviousStepLooped;
    }

    /**
     * @brief Repeats n previous steps until callback returns true
     *
     * @param steps How many steps to go back each time
     * @param callback Function to evaluate whether (true) or not (false) to go back
     * @return Sequence& returns the sequence so chaining can continue
     */
    template <typename F>
    Sequence &repeatPreviousStepsUntil(int steps, F callback)
    {
        if (paused)
        {
            return *this;
        }
        if (checkedSteps == sequenceStep)
        {
            // Should run before callback is checked
            timesPreviousStepLooped += 1;
            if (callback())
            {
                moveSteps(1);
                timesPreviousStepLooped = 0;
            }
            else
            {
                moveSteps(-steps);
            }
        }
        checkedSteps += 1;
        return *this;
    }

    /**
     * @brief Repeats n previous steps n times
     *
     * @param steps How many steps to go back each time
     * @param times How many times to go back
     * @return Sequence& returns the sequence so chaining can continue
     */
    Sequence &repeatPreviousStepsTimes(int steps, int times)
    {
        if (paused)
        {
            return *this;
        }

        return repeatPreviousStepsUntil(steps, [&]() { //
            return timesPreviousStepLooped >= times;
        });
    }

    /**
     * Will run the code provided for the specified number of milliseconds
     * and then continue to the next step
     *
     * @tparam F
     * @param durationMs how long the callback function should run
     * @param callback function to run for the durationMs
     * @return Sequence&
     */
    template <typename F>
    Sequence &thenRunFor(long durationMs, F callback)
    {
        if (paused)
        {
            return *this;
        }

        if (checkedSteps == sequenceStep)
        {
            if (timer.isFinished(durationMs))
            {
                moveSteps(1);
            }
            else
            {
                callback();
            }
        }
        checkedSteps += 1;
        return *this;
    }

    /**
     * Will wait until the specified time has passed and then go to next sequence step.
     * This function is non-blocking and shares nothing with the Arduino delay() function.
     * While the sequence will not continue until the specified time has passed, the rest of
     * your code will continue to run normally. This function uses millis() behing the scenes.
     *
     * @param delayMs how many milliseconds to wait until continuing to the next step in the sequence
     * @return Sequence&
     */
    Sequence &delay(long delayMs)
    {
        if (paused)
        {
            return *this;
        }
        return thenRunFor(delayMs, [&] {});
    }

    /**
     * @brief Restart sequence from beginning when sequence has finished executing
     *
     * @return Sequence&
     */
    Sequence &loop()
    {
        if (paused)
        {
            return *this;
        }

        if (checkedSteps == sequenceStep)
        {
            endSequenceShouldReturnTrueBetweenLoops = true;
            shouldLoop = true;
            sequenceStep += 1;
        }

        checkedSteps += 1;
        return *this;
    }

    /**
     * Restart sequence from beginning when sequence has finished executing
     * until the callback returns true
     *
     * @param callback function to run until it returns true
     * @return template <typename F>&
     */
    template <typename F>
    Sequence &loopUntil(F callback)
    {
        if (paused)
        {
            return *this;
        }

        if (checkedSteps == sequenceStep)
        {
            endSequenceShouldReturnTrueBetweenLoops = false;
            shouldLoop = !callback();
            sequenceStep += 1;
        }

        checkedSteps += 1;
        return *this;
    }

    /**
     * Will prevent endSequence from returning true until the
     * specified number of times has been looped
     * When the specified number of times has been looped,
     * the sequence will start executing from the first step
     * just like with a normal loop
     * @param times how many times to loop before endOfSequence() returns true
     * @param restartAfterCompletion true
     *  Will restart the sequence from the first step
     *  after the specified number of times has been looped.
     *  During the last update after the sequence has run the
     *  specified number of times, endOfSequence will return true
     *  before the sequence restarts and counts from zero.
     *  Leave this as true if you do not intend to manually reset the
     *  sequence between uses.
     */
    Sequence &loopTimes(int times, bool restartAfterCompletion)
    {
        if (paused)
        {
            return *this;
        }

        if (checkedSteps == sequenceStep)
        {
            endSequenceShouldReturnTrueBetweenLoops = false;
            shouldLoop = true;
            sequenceStep += 1;
            timesToLoop = times;
            // timesLooped should be incremented before the check
            timesLooped += 1;
            if (timesLooped == timesToLoop)
            {
                endSequenceShouldReturnTrueBetweenLoops = true;
                // This will allow the sequence to restart from the first step
                // and simply return true after the specified number of times has been looped
                // This way there is a queue for when the fixed number of loops has been completed
                // and the sequence can be restarted from the first step just like with a normal
                // loop. This results in reusable sequences that do not require a manual restart.
                shouldLoop = restartAfterCompletion;
                timesToLoop = 0;
                timesLooped = 0;
            }
        }

        checkedSteps += 1;
        return *this;
    }

    /**
     * Returns true if the sequence has finished executing and should be restarted
     * to continue from the first step
     *
     * @return true if the sequence has finished executing
     */
    bool hasFinished()
    {
        return sequenceHasFinished;
    }

    /**
     * @brief Mark end of sequence and return true if sequence has finished executing
     *
     * @return true if the sequence is finished executing all the step
     * or false if the sequence is not finished executing all the steps
     * or if conditions in loopTimes() or loopUntil() have not been met
     */
    bool endOfSequence()
    {
        if (paused)
        {
            return true;
        }

        if (numSteps < checkedSteps)
        {
            numSteps = checkedSteps;
        }
        checkedSteps = 0;

        sequenceHasFinished = sequenceStep == numSteps;

        if (sequenceHasFinished)
        {
            if (shouldLoop)
            {
                sequenceStep = 0;
                return endSequenceShouldReturnTrueBetweenLoops;
            }
            else
            {
                // The sequence execution
                // is complete and there is
                // no looping. This block will
                // run until the sequence is
                // restarted manually by the user
                return true;
            }
        }

        return false;
    }
};

#endif
