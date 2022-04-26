#include <Arduino.h>
#include <Wire.h>
#include <Zumo32U4.h>
#include <PololuMenu.h>
#include <Vector.h>
#include <Zumo32U4Encoders.h>
#include <Zumo32U4Motors.h>
#include "Timer.h"

// til testing av switch case og funksjoner

Zumo32U4Buzzer buzzer;
Zumo32U4LCD display;
Zumo32U4Encoders encoder;
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;
Timer timer;

float account_balance = 8615; // money in the bank

float amphere = 1200; // current value in battery
// unsigned longs used for timing purposes
unsigned long wait = 0;
unsigned long now;
unsigned long interval = 0;
unsigned long count = 0;
unsigned long pattern = 0;
unsigned long startMillis = 0;
unsigned long n = 0;
unsigned long displayrate = 0;
unsigned long startChargingDisplayrate;
unsigned long currentChargingDisplayrate;
const unsigned long period = 1000;
unsigned long startDefaultDisplayrate;
unsigned long currentDefaultDisplayrate;
const unsigned long showtime = 1000;
// returned values
float highestSpeed = 0;                    // highestspeed in 60 sec interval
float vel = 0;                             // value returning speed calculation
float sTotal = 0;                          // value returning total distance
float timeAtHighSpeeds = 0;                // value returning amount of time at 70% or higher speed
float averageSpeed = 0;                    // value returning averagespeed in 60sec interval
const float seventypercentofmaxspeed = 70; // value defining the 70%
float lastMinutesHighestSpeed = 0;
float lastMinutesAverageSpeed = 0;
float lastMinutestimeAtHighSpeeds = 0;
int amountOfNumbersInAverage = 0;
int amountOfSpeedsRecorded = 0;
int amountOfSpeedsRecordedOverSeventyPercent = 0;
int percentofBatteryleft = 0;          // amount of percent left on battery
int reversecharge = 20;                // charge in reverse charging mode
int percents = 100;                    // value used in calculationOfBatteryLeft
float batteryhealth = 100;             // current battery health
int amountofchargingcycles = 0;        // amount of times battery has charged
int timesreachedfivepercentcharge = 0; // amount of times battery has reached five percent
const int ELEMENT_COUNT_MAX = 1;       // value used to creat vector
float batterylevel = 1;
byte flag = 1; // used to check amount of charges
byte lag = 1;  // used to check emergency charging
int amountOfMoneyUsedInCharge = 0;
byte cleanenergy = 1;
int costCharging = 30;
long currentsteps = 0;
long lastSteps = 0;
long steps = 0;

const char tenpercentsound[] PROGMEM =
    "!af";

const char fivepercentsound[] PROGMEM =
    "!af";

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  randomSeed((unsigned int)millis());
  display.init();
  encoder.init();
  startMillis = millis();
  startDefaultDisplayrate = millis();
  Serial.println("In setup");
}

// Increase refresh rate if speed
// is too low to be detected
int refreshRateTimesPerSec = 10;
long stepCounter = 0;
long stepsPerSecond = 0;

long getStepsFromPoluluLib()
{
  // TODO: Implement this
  
}

void updateStepsPerSecond()
{
  if (timer.loopWait(1000 / refreshRateTimesPerSec))
  {
    long currentSteps = getStepsFromPoluluLib();
    stepsPerSecond = (currentSteps - stepCounter) * refreshRateTimesPerSec;
    stepCounter = currentSteps;
  }
}

float getSpeed(int refreshRate = 10)
{
  updateStepsPerSecond();

  return stepsPerSecond;

  /*int startCounts = encoder.getCountsRight();
  unsigned long now = millis();
  while (millis() < now + refreshRate)
  {
  }
  int counts = encoder.getCountsAndResetRight() - startCounts;
  float cm = counts / 195.0;
  vel = (cm / refreshRate) * 1000;
  return vel;*/
} // end float get speed

float getDistance(float speed)
{
  unsigned long now = millis();
  float sInterval = speed * (now / 1000 - interval / 1000);
  interval = millis();
  sTotal = sTotal + sInterval;
  return sTotal;
}

void count60secondinterval()
{
  int startCounts = encoder.getCountsRight();
  if (startCounts != 0)
  {
    unsigned long now = millis();
    int timeinterval = (now / 1000) - (count / 1000);
    count = millis();
    pattern = pattern + timeinterval;
  }
}

float getMaxSpeed(float speed)
{
  if (speed > highestSpeed)
  {
    highestSpeed = speed;
  }

  if (pattern >= 60)
  {
    lastMinutesHighestSpeed = highestSpeed;
    highestSpeed = 0;
    return lastMinutesHighestSpeed;
  }

  else
  {
    return highestSpeed;
  }
}

float getAverageSpeed(float speed)
{
  amountOfNumbersInAverage++;
  n = n + speed;
  averageSpeed = n / amountOfNumbersInAverage;
  if (pattern >= 60)
  {
    lastMinutesAverageSpeed = averageSpeed;
    n = 0;
    averageSpeed = 0;
    amountOfNumbersInAverage = 0;
    return lastMinutesAverageSpeed;
  }
  else
  {
    return averageSpeed;
  }
}

float getAmphere(float averageSpeed, float Distance)
{
  float y = 2 * averageSpeed + 10;
  amphere = amphere * 3600;
  amphere = amphere - y * (Distance / 1000);
  amphere = amphere / 3600;
  return amphere;
} // end void getAmphere

float getspeedshigherthan70percent(float speed)
{
  amountOfSpeedsRecorded++;
  if (speed >= seventypercentofmaxspeed)
  {
    amountOfSpeedsRecordedOverSeventyPercent++;
  }
  timeAtHighSpeeds = (60 * (amountOfSpeedsRecordedOverSeventyPercent * 100)) / amountOfSpeedsRecorded / 100;
  if (pattern >= 60)
  {
    lastMinutestimeAtHighSpeeds = timeAtHighSpeeds;
    amountOfSpeedsRecorded = 0;
    amountOfSpeedsRecordedOverSeventyPercent = 0;
    timeAtHighSpeeds = 0;
    return lastMinutestimeAtHighSpeeds;
  }
  else
  {
    return timeAtHighSpeeds;
  }
}

void resultsOf60SecondInterval()
{
  count60secondinterval();
  if (pattern >= 60)
  {
    getMaxSpeed(getSpeed(10));
    getAverageSpeed(getSpeed(10));
    getspeedshigherthan70percent(getSpeed(10));
    pattern = 0;
  }
}

int calculationOfBatteryLeft(float amphere)
{
  percentofBatteryleft = (amphere / 1200) * 100;
  return percentofBatteryleft;
}

void chargingmode()
{
}

void hiddencharging()
{
}

void emergencycharge()
{
}

void batteryhealthchange(
    float chargincycles,
    float fivepercenttimes,
    float sixtysecaverage,
    float sixtysecspeed,
    float seventypercentofmax)
{
  float malfunction = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 100));

  if (malfunction <= 5)
  {
    batteryhealth = batteryhealth - (chargincycles * 2 + fivepercenttimes * 4 + sixtysecaverage / 10000) - ((sixtysecspeed * seventypercentofmax) / 750) - ((batteryhealth * 50) / 100);
  }
  if (malfunction > 5)
  {
    batteryhealth = batteryhealth - (chargincycles * 2 + fivepercenttimes * 2 + sixtysecaverage / 10000) - ((sixtysecspeed * seventypercentofmax) / 75000);
  }
}

void batterylevelhchange(float batteryhealth)
{
  if (batteryhealth <= 35)
  {
    batterylevel = 1;
  }
  if (batteryhealth <= 10)
  {
    batterylevel = 0;
  }
}

void tenpercentbatteryleft()
{
  display.clear();                  // clearer displayet
  display.setCursor(0, 0);          // setter linja som skriver i displayet til koordinat 0x,0y
  display.print("10 percent left"); // printer beskjeden i displayet
  buzzer.playFromProgramSpace(tenpercentsound);
  ledYellow(1);
  buzzer.stopPlaying();
  ledYellow(0);
}

void fivepercentleft()
{
  ledRed(1);
  display.clear();                 // clearer displayet
  display.setCursor(0, 0);         // setter linja som skriver i displayet til koordinat 0x,0y
  display.print("5 percent left"); // printer beskjeden i displayet
  buzzer.playFromProgramSpace(fivepercentsound);
  unsigned long currentMillis = millis();
  if (currentMillis - startMillis >= 15000)
  {
    motors.setSpeeds(0, 0);
    buzzer.playFrequency(6000, 100, 15);
  }
  startMillis = millis();
}

void normalDriving()
{
  count60secondinterval();
  float drive = getSpeed(10);
  float distance = getDistance(drive);
  float average = getAverageSpeed(drive);
  float highest = getMaxSpeed(drive);
  float timeAtHighSpeeds = getspeedshigherthan70percent(drive);
  Serial.print("Speed");
  Serial.println(drive);
  Serial.print("Distance");
  Serial.println(distance);
  Serial.print("average");
  Serial.println(average);
  Serial.print("highest");
  Serial.println(highest);
  Serial.print("70%");
  Serial.println(timeAtHighSpeeds);
  Serial.println("Normal driving");
  if (pattern >= 60)
  {
    pattern = 0;
  }
  currentDefaultDisplayrate = millis();
  if (currentDefaultDisplayrate - startDefaultDisplayrate >= showtime)
  {
    display.clear();
    display.print(drive);
    display.setCursor(0, 1);
    display.print(distance);
    Serial.println("2 second passed");
  }
  float battery = getAmphere(drive, distance);
  percents = calculationOfBatteryLeft(battery);
  if (percents <= 10 && percents > 5)
  {
    tenpercentbatteryleft();
  }
  if (percents <= 5)
  {
    fivepercentleft();
  }
  batteryhealthchange(amountofchargingcycles, timesreachedfivepercentcharge, lastMinutesAverageSpeed, lastMinutesHighestSpeed, lastMinutestimeAtHighSpeeds);
  now = millis();
  if (now - displayrate >= 10000)
  {
    display.clear();
    display.print(percents);
    display.setCursor(0, 1);
    display.print(amountofchargingcycles);
    display.setCursor(0, 2);
    display.print(batteryhealth);
    displayrate = millis();
    Serial.println("10 second passed");
  }
}

void loop()
{
  // put your main code here, to run repeatedly:
  motors.setSpeeds(75, 75);
  int state = 0;
  Serial.println("In switch");
  if (buttonA.getSingleDebouncedPress())
  {
    Serial.println("in button A");
    state = 1;
  }
  if (buttonB.getSingleDebouncedPress())
  {
    state = 2;
  }

  if (batterylevel <= 0 || amphere <= 0)
  {
    amphere = 0;
    batterylevel = 0;
    state = 4;
  }

  switch (state)
  {

  case 1: // charging mode
  {
    display.clear();
    display.print("Your in charging mode");
    Serial.println("Your in charging mode");

    while (account_balance >= 0 && amphere < 1200)
    {
      if (buttonA.getSingleDebouncedPress()) // regular charging
      {
        if (flag == 1)
        {
          amountofchargingcycles++;
          flag = 0;
        }

        if (cleanenergy == 1)
        {
          costCharging = 25;
        }

        if (cleanenergy != 1)
        {
          costCharging = 50;
        }
        amphere = amphere + 120;
        account_balance = account_balance - costCharging;
        amountOfMoneyUsedInCharge = amountOfMoneyUsedInCharge + costCharging;
        if (amphere > 1200)
        {
          amphere = 1200;
        }
        ledYellow(1);
      }

      if (buttonB.getSingleDebouncedPress()) // battery service
      {
        batterylevel = batterylevel + 1 / 100 * 15;
        batteryhealth = batteryhealth + 100 / 100 * 15;
        account_balance = account_balance - 200;
        amountOfMoneyUsedInCharge = amountOfMoneyUsedInCharge + 200;
        if (batterylevel > 1)
        {
          batterylevel = 1;
        }
      }
      if (buttonC.getSingleDebouncedPress()) // battery change
      {
        batterylevel = 1;
        account_balance = account_balance - 300;
        amountOfMoneyUsedInCharge = amountOfMoneyUsedInCharge + 300;
      }
      currentChargingDisplayrate = millis();
      if (currentChargingDisplayrate - startChargingDisplayrate >= period)
      {
        display.clear();
        display.print(batterylevel);
        display.setCursor(0, 1);
        display.print(amountOfMoneyUsedInCharge);
        display.setCursor(0, 2);
        display.print(account_balance);
        startChargingDisplayrate = millis();
        Serial.println(amphere);
        Serial.println(batteryhealth);
        Serial.println(account_balance);
        Serial.println(percentofBatteryleft);
      }
    }
    flag = 1;
    ledYellow(0);
  }
  break;

  case 2: // reverse charging
  {
    display.clear();
    display.print("Your in case 2");
  }
  break;

  case 3: // dead battery
    ledGreen(1);
    while (true)
    {
    }

    break;

  default: // driving without anything
  {
    normalDriving();
  }
  break;
  } // end switch case
} // end void loop