#ifndef Timer_h
#define Timer_h
#include <Arduino.h>

class Timer
{
private:
    unsigned long startTime;

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
        startTime = micros();
    }

    /**
     * @brief Get the time in microseconds since the timer was started or reset
     *
     */
    unsigned long getElapsedTimeMicros()
    {
        return micros() - startTime;
    }
    
    /**
     * @brief Get the time in milliseconds since the timer was started or reset
     *
     */
    unsigned long getElapsedTime()
    {
        return getElapsedTimeMicros() / 1000;
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
        if (isFinished(durationMs))
        {
            reset();
            return true;
        }
        return false;
    }
};

#endif
