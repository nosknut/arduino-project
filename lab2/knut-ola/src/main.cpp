// The "//-LaTeX:SectionName;" anchors
// for auto generated repors.
// See: ../report/report.tex
// NB!
// There can not be a /* type comment on the
// line directly below an anchor.

//-LaTeX:imports;
#include <Arduino.h>
#include <Ramp.h>
#include <Servo.h>
#include <Range.h>
#include <ApplicationConfig.h>
#include <Photoresistor.h>
//-LaTeX:End_Section;

//-LaTeX:CLAMPING;
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
//-LaTeX:End_Section;

//-LaTeX:Motor_Control;
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

void setupMotor(const MotorConfig motorConfig)
{
    pinMode(motorConfig.enablePin, OUTPUT);
    pinMode(motorConfig.in1Pin, OUTPUT);
    pinMode(motorConfig.in2Pin, OUTPUT);
}
//-LaTeX:End_Section;

//-LaTeX:Servo_Control;
void setServoPosition(const ServoConfig servoConfig, Servo servo, const int position)
{
    const int targetPosition = clampAngle(position, servoConfig.angleLimits);
    servo.write(targetPosition);
}
//-LaTeX:End_Section;

//-LaTeX:Emergency_Stop;
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
//-LaTeX:End_Section;

//-LaTeX:setup;
ramp servoRamp;
Servo servo;
Photoresistor photoresistor(appConfig.photoresistorPin);

void setup()
{
    Serial.begin(appConfig.baudRate);
    setupEmergencyStop();
    photoresistor.setup();
    servo.attach(appConfig.servoConfig.servoPin);
    setupMotor(appConfig.motorConfig);
    pinMode(appConfig.potmeterPin, INPUT);
}
//-LaTeX:End_Section;

//-LaTeX:Center_Potmeter;
const int centerAnalogInput(const int value)
{
    return map(value, 0, 1023, -255, 255);
}
//-LaTeX:End_Section;

//-LaTeX:Exercise_3;

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
    MotorConfig motorConfig = appConfig.motorConfig;
    rampMotorTo(motorConfig, rampTime, 255, MotorDirection::CLOCKWISE);
    rampMotorTo(motorConfig, rampTime, 0, MotorDirection::CLOCKWISE);
    rampMotorTo(motorConfig, rampTime, 255, MotorDirection::COUNTERCLOCKWISE);
    rampMotorTo(motorConfig, rampTime, 0, MotorDirection::COUNTERCLOCKWISE);
}
//-LaTeX:End_Section;

//-LaTeX:Exercise_4;
// Exercise 4
void centeredPotmeterMotorControlLoop()
{
    const int potmeterValue = analogRead(appConfig.potmeterPin);
    setMotorSpeed(appConfig.motorConfig, centerAnalogInput(potmeterValue));
}
//-LaTeX:End_Section;

//-LaTeX:Exercise_6;
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
//-LaTeX:End_Section;

//-LaTeX:Exercise_7;
// Exercise 7
void directPotmeterServoControlLoop()
{
    const int potmeterValue = analogRead(appConfig.potmeterPin);
    const int servoPosition = map(potmeterValue, 0, 1023, 0, 180);
    setServoPosition(appConfig.servoConfig, servo, servoPosition);
}
//-LaTeX:End_Section;

//-LaTeX:Exercise_8;
// Exercise 8
void speedIndicator()
{
    const int potmeterValue = analogRead(appConfig.potmeterPin);
    const int motorSpeed = centerAnalogInput(potmeterValue);
    const int servoPosition = map(abs(motorSpeed), 0, 255, 0, 180);
    setServoPosition(appConfig.servoConfig, servo, servoPosition);
    setMotorSpeed(appConfig.motorConfig, motorSpeed);
}
//-LaTeX:End_Section;

//-LaTeX:Exercise_9;
// Exercise 9
void photoresistorSpeedControl()
{
    const int photoresistorValue = photoresistor.read();
    const int motorSpeed = centerAnalogInput(photoresistorValue);
    const int servoPosition = map(abs(motorSpeed), 0, 255, 0, 180);
    setServoPosition(appConfig.servoConfig, servo, servoPosition);
    setMotorSpeed(appConfig.motorConfig, motorSpeed);
}
//-LaTeX:End_Section;

//-LaTeX:loop;
void loop()
{
    // Uncomment code to run an exercise
    // NB! These procedures are not designed for concurrent runs

    // Exercise 3
    // fadeLoop();

    // Exercise 4
    // centeredPotmeterMotorControlLoop();

    // Exercise 6
    // potmeterFadeLoop();

    // Exercise 7
    // directPotmeterServoControlLoop();

    // Exercise 8
    // speedIndicator();

    // Exercise 9
    // photoresistorSpeedControl();
}
//-LaTeX:End_Section;
