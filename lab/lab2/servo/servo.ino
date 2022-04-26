#include <Servo.h>

const int servoPin = 11;
const int tid = 1000;

Servo servo;

void setup()
{
    servo.attach(servoPin);
}

void loop()
{
    servo.write(0);
    delay(tid);
    servo.write(90);
    delay(tid);
    servo.write(180);
    delay(tid);
}