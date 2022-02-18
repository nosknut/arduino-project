#include <Arduino.h>
#include <Ramp.h>
#include <Servo.h>
#include <Range.h>
#include <ApplicationConfig.h>

class Photoresistor
{
    // Start with oposite values
    Range limits = Range(1023, 0);
    const int pin;

public:
    Photoresistor(const int pin) : pin(pin)
    {
    }

    int mapWithinLimits(const int value)
    {
        return map(value, limits.minValue, limits.maxValue, 0, 1023);
    }
    int read()
    {
        const int value = analogRead(pin);
        // Always update the limits
        if (value < limits.minValue)
        {
            limits.minValue = value;
        }
        if (value > limits.maxValue)
        {
            limits.maxValue = value;
        }
        return mapWithinLimits(value);
    }

    void setup()
    {
        pinMode(pin, INPUT);
    }
};

const int clampAngle(const int value, const Range range)
{
    return min(max(value, range.minValue), range.maxValue);
}

const int clampSpeed(const int value, const Range range)
{
    if (value < range.minValue)
    {
        return 0;
    }
    if (value > range.maxValue)
    {
        return range.maxValue;
    }
    return value;
}

enum class MotorDirection
{
    CLOCKWISE,
    COUNTERCLOCKWISE,
};

void stopMotor(const MotorConfig motorConfig)
{
    digitalWrite(motorConfig.in1Pin, LOW);
    digitalWrite(motorConfig.in2Pin, LOW);
}

void setMotorDirection(const MotorConfig motorConfig, const MotorDirection direction)
{
    const int in1Pin = motorConfig.in1Pin;
    const int in2Pin = motorConfig.in2Pin;

    if (direction == MotorDirection::CLOCKWISE)
    {
        digitalWrite(in1Pin, HIGH);
        digitalWrite(in2Pin, LOW);
    }
    else
    {
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, HIGH);
    }
}

/**
    @param speed 0 to 255
*/
void setMotorSpeed(const MotorConfig motorConfig, const int speed, const MotorDirection direction)
{
    const int limitedSpeed = clampSpeed(abs(speed), motorConfig.speedLimits);
    setMotorDirection(motorConfig, direction);
    analogWrite(motorConfig.enablePin, limitedSpeed);
}

/**
    @param speed -255 to 255
*/
void setMotorSpeed(const MotorConfig motorConfig, const int speed)
{
    setMotorSpeed(
        motorConfig,
        abs(speed),
        speed > 0 ? MotorDirection::CLOCKWISE : MotorDirection::COUNTERCLOCKWISE);
}

void setServoPosition(const ServoConfig servoConfig, Servo servo, const int position)
{
    const int targetPosition = clampAngle(position, servoConfig.angleLimits);
    servo.write(targetPosition);
}

// Exercise 5
void emergencyStopInterrupt()
{
    while (digitalRead(appConfig.emergencyStopPin) == HIGH)
    {
        stopMotor(appConfig.motorConfig);
    }
}

// Exercise 5
void setupEmergencyStop()
{
    const int pin = appConfig.emergencyStopPin;
    pinMode(pin, INPUT_PULLUP);
    // Emergency stops should be normally closed (NC), so that cutting the wire
    // Causes the motor to stop.
    // Because the pinMode is PULLUP, that means that the pin should be LOW when
    // the emergancy stop is not activated.
    attachInterrupt(digitalPinToInterrupt(pin), emergencyStopInterrupt, RISING);
}

ramp servoRamp;
Servo servo;
Photoresistor photoresistor(appConfig.photoresistorPin);

void setupMotor(const MotorConfig motorConfig)
{
    pinMode(motorConfig.enablePin, OUTPUT);
    pinMode(motorConfig.in1Pin, OUTPUT);
    pinMode(motorConfig.in2Pin, OUTPUT);
}

void setup()
{
    Serial.begin(appConfig.baudRate);
    setupEmergencyStop();
    photoresistor.setup();
    servo.attach(appConfig.servoConfig.servoPin);
    setupMotor(appConfig.motorConfig);
    pinMode(appConfig.potmeterPin, INPUT);
}

const int centerAnalogInput(const int value)
{
    return map(value, 0, 1023, -255, 255);
}

/*
    This does not guard the motor from directions switching during full speed
    Changing speed from 255 CW to 255 CCW could cause damage to the motor as
    directional change is instant
*/
void rampMotorTo(const MotorConfig motorConfig, const int rampTime, const int speed, const MotorDirection direction)
{
    servoRamp.go(speed, rampTime);
    while (servoRamp.isRunning())
    {
        servoRamp.update();
        setMotorSpeed(appConfig.motorConfig, servoRamp.getValue(), direction);
    }
}

// Exercise 3
void fadeLoop()
{
    const int rampTime = 2000;
    rampMotorTo(appConfig.motorConfig, rampTime, 255, MotorDirection::CLOCKWISE);
    rampMotorTo(appConfig.motorConfig, rampTime, 0, MotorDirection::CLOCKWISE);
    rampMotorTo(appConfig.motorConfig, rampTime, 255, MotorDirection::COUNTERCLOCKWISE);
    rampMotorTo(appConfig.motorConfig, rampTime, 0, MotorDirection::COUNTERCLOCKWISE);
}

// Exercise 4
void centeredPotmeterMotorControlLoop()
{
    const int potmeterValue = analogRead(appConfig.potmeterPin);
    setMotorSpeed(appConfig.motorConfig, centerAnalogInput(potmeterValue));
}

// Exercise 6
void potmeterFadeLoop()
{
    // 600ms/60deg
    // (180/60) * 600 = 1800ms
    const int rampTime = 1800;
    servoRamp.go(180, rampTime);
    while (servoRamp.isRunning())
    {
        servoRamp.update();
        setServoPosition(appConfig.servoConfig, servo, servoRamp.getValue());
    }
    servoRamp.go(0, rampTime);
    while (servoRamp.isRunning())
    {
        servoRamp.update();
        setServoPosition(appConfig.servoConfig, servo, servoRamp.getValue());
    }
}

// Exercise 7
void directPotmeterServoControlLoop()
{
    const int potmeterValue = analogRead(appConfig.potmeterPin);
    const int servoPosition = map(potmeterValue, 0, 1023, 0, 180);
    setServoPosition(appConfig.servoConfig, servo, servoPosition);
}

// Exercise 8
void speedIndicator()
{
    const int potmeterValue = analogRead(appConfig.potmeterPin);
    const int motorSpeed = centerAnalogInput(potmeterValue);
    const int servoPosition = map(abs(motorSpeed), 0, 255, 0, 180);
    setServoPosition(appConfig.servoConfig, servo, servoPosition);
    setMotorSpeed(appConfig.motorConfig, motorSpeed);
}

// Exercise 9
void photoresistorSpeedControl()
{
    const int photoresistorValue = photoresistor.read();
    const int motorSpeed = centerAnalogInput(photoresistorValue);
    const int servoPosition = map(abs(motorSpeed), 0, 255, 0, 180);
    setServoPosition(appConfig.servoConfig, servo, servoPosition);
    setMotorSpeed(appConfig.motorConfig, motorSpeed);
}

void loop()
{
    speedIndicator();
}
