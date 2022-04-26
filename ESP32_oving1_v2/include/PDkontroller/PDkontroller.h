#ifndef PDkontroller_h
#define PDkontroller_h
#include <Arduino.h>
#include <Zumo32U4.h>

class PDkontroller
{
private:
    int wanted_value = 2000;
    int topSpeed = 400;
    int last_error = 0;
    int kp = 1;

public:
    PDkontroller(const int sensorData)
    {
        pd(sensorData);
    }
    void pd(const int posisjon)
    {
        int error = posisjon - wanted_value;
        int d_ledd = 2 * (error - last_error);
        int speedDifference = error * kp + d_ledd;
        int leftSpeed = 400 - speedDifference;
        int rightSpeed = 400 + speedDifference;
        // bruker constrain så verdiene holder seg mellom 0 og 400
        leftSpeed = constrain(leftSpeed, 0, topSpeed);
        rightSpeed = constrain(rightSpeed, 0, topSpeed);
        last_error = error;
    }
};

#endif