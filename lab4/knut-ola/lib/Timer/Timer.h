#ifndef Timer_h
#define Timer_h
#include <Arduino.h>

class Timer
{
private:
    long startTime;

public:
    Timer()
    {
        reset();
    }
    void reset()
    {
        startTime = millis();
    }

    unsigned long getElapsedTime()
    {
        return millis() - startTime;
    }

    /*
        Will check if the time since last reset()
        call is greater than the given time
        Note that while loopWait() automatically resets after
        the given time, this function does not
      */
    bool isFinished(const unsigned long durationMs)
    {
        return getElapsedTime() >= durationMs;
    }
    /*
    Will return false until the given time has passed
    Then it will return true and start counting down the same amount again
    Example:
        Timer timer;
        while(true) {
            if(timer.isTimePassed(1000)) {
                // Will run every 1000ms
            }
        }
    */
    bool loopWait(const unsigned long durationMs)
    {
        if (isFinished(durationMs))
        {
            reset();
            return true;
        }
        return false;
    }
};

#endif
