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
    bool shouldLoopASetNumberTimes = false;

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
        Serial.println("Moving steps");
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
            Serial.println("Paused");
            return *this;
        }
        Serial.print(checkedSteps);
        Serial.print(" ");
        Serial.println(sequenceStep);
        if (checkedSteps == sequenceStep)
        {
            callback();
            moveSteps(1);
        }
        checkedSteps += 1;
        return *this;
    }

    // Will reset the sequence to the first step
    void restart()
    {
        checkedSteps = 0;
        sequenceStep = 0;
        timesLooped = 0;
        timesPreviousStepLooped = 0;
        paused = false;
    }

    void start()
    {
        restart();
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

        if (callback())
        {
            moveSteps(1);
        }
        else
        {
            timesPreviousStepLooped += 1;
            moveSteps(-steps);
        }
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
            delay(durationMs);
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

        shouldLoopASetNumberTimes = false;
        if (checkedSteps == sequenceStep)
        {
            shouldLoop = true;
        }
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

        shouldLoopASetNumberTimes = true;
        timesToLoop = times;
        if (timesLooped == timesToLoop)
        {
            timesLooped = 0;
        }
        else
        {
            timesLooped += 1;
            loop();
        }
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
            return false;
        }

        if (numSteps < checkedSteps)
        {
            numSteps = checkedSteps;
        }
        checkedSteps = 0;

        sequenceHasFinished = (numSteps != 0) && (sequenceStep == numSteps);

        if (sequenceHasFinished && shouldLoop)
        {
            shouldLoop = false;
            restart();
        }

        bool hasFinishedLooping = !shouldLoopASetNumberTimes || (timesLooped == timesToLoop);

        return sequenceHasFinished && hasFinishedLooping;
    }
};

#endif
