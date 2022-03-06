#ifndef Sequence_h
#define Sequence_h
#include <Arduino.h>
#include <Timer.h>

class Sequence
{
private:
    int timesPreviousStepLooped = 0;

    int timesLooped = 0;
    int timesToLoop = 0;

    // Keeps track of what step we should be executing
    int sequenceStep = 0;
    // Keeps track of how many steps we have registered
    int checkedSteps = 0;
    // Keeps track of how many steps we registered last time
    // we ran the sequence (the last known value of checkedSteps)
    // Used to determine how many steps are in the sequence
    // and by extension, if the sequence execution has finished executing.
    int numSteps = 0;
    // Will be true after the last step has executed and during the execution
    // of the first step after a sequence restart.
    bool sequenceHasFinished = false;
    bool shouldLoop = false;
    bool paused = false;

public:
    Timer timer;
    void moveSteps(int deltaSteps)
    {
        sequenceStep += deltaSteps;
        timer.reset();
    }
    // Will run the code provided and go to next sequence step
    // if the function returns true
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

    // Will run the code provided and go to next sequence step
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

    // Will reset the sequence to the first step
    void reset()
    {
        checkedSteps = 0;
        sequenceStep = 0;
        timesLooped = 0;
        timesPreviousStepLooped = 0;
        paused = false;
        sequenceHasFinished = false;
    }

    void start()
    {
        reset();
    }

    void pause()
    {
        paused = true;
    }

    void resume()
    {
        paused = false;
    }

    int getTimesSequenceLooped()
    {
        return timesLooped;
    }

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
            // SHould run before callback is checked
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

    // Will run the code provided for the specified number of milliseconds
    // and then continue to the next step
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

    // Will wait until the specified time has passed and then go to next sequence step
    Sequence &delay(long delayMs)
    {
        if (paused)
        {
            return *this;
        }
        return thenRunFor(delayMs, [&] {});
    }

    // Restart sequence from beginning when sequence has finished executing
    Sequence &loop()
    {
        if (paused)
        {
            return *this;
        }

        if (checkedSteps == sequenceStep)
        {
            shouldLoop = true;
            sequenceStep += 1;
        }

        checkedSteps += 1;
        return *this;
    }

    // Restart sequence from beginning when sequence has finished executing
    // until the callback returns true
    template <typename F>
    Sequence &loopUntil(F callback)
    {
        if (paused)
        {
            return *this;
        }

        if (checkedSteps == sequenceStep)
        {
            shouldLoop = !callback();
            sequenceStep += 1;
        }

        checkedSteps += 1;
        return *this;
    }

    // Will prevent endSequence from returning true until the
    // specified number of times has been looped
    // When the specified number of times has been looped,
    // the sequence will start executing from the first step
    // just like with a normal loop
    Sequence &loopTimes(int times)
    {
        if (paused)
        {
            return *this;
        }

        if (checkedSteps == sequenceStep)
        {
            shouldLoop = true;
            sequenceStep += 1;
            timesToLoop = times;
            // timesLooped should b incremented before the check
            timesLooped += 1;
            if (timesLooped == timesToLoop)
            {
                shouldLoop = false;
                timesToLoop = 0;
                timesLooped = 0;
            }
        }

        checkedSteps += 1;
        return *this;
    }

    // Returns true if the sequence has finished executing and should be restarted
    // to continue from the first step
    bool hasFinished()
    {
        return sequenceHasFinished;
    }

    // Mark end of sequence and return true if sequence has finished executing
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
            }
            else
            {
                // The sequence execution
                // is complete and there is
                // no looping. This block will
                // run until the sequence is
                // restarted manually by the user
            }
            return true;
        }

        return false;
    }
};

/*
Example usage:

// Sequences
Sequence redSequence;
Sequence yellowSequence;

void loop()
{
    redSequence
        .then([&] { //
            ledRed(true);
        })
        .delay(2000)
        .then([&] { //
            ledRed(false);
        })
        .delay(2000)
        .loop()
        .endOfSequence();

    yellowSequence
        .then([&] { //
            ledYellow(true);
        })
        .delay(200)
        .then([&] { //
            ledYellow(false);
        })
        .delay(200)
        .loop()
        .endOfSequence();
}
*/

#endif
