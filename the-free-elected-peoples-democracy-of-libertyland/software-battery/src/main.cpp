// include libraries
#include <Arduino.h>
#include <Wire.h>
#include <Zumo32U4.h>
#include <PololuMenu.h>
#include <Zumo32U4Encoders.h>
#include <Zumo32U4Motors.h>
#include <Timer.h>
#include <EEPROM.h>

// define parts of libraries
Zumo32U4Buzzer buzzer;
Zumo32U4LCD display;
Zumo32U4Encoders encoders;
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;
Timer timer;

int account_balance = 8615; // money avaliable for charging and batterychanges
// unsigned longs used for timing purposes
unsigned long distanceInterval = 0;
unsigned long previousOneMinuteCounting = 0;
unsigned long displayrate = 0;
unsigned long startChargingDisplayrate = 0;
unsigned long changebatteryhealthrate = 0;
unsigned long changeAmphereRate = 0;
unsigned long changePositiveAmphereRate = 0;
unsigned long start10perDisplayrate = 0;
float sTotal = 0;                                   // value returning total distance
byte oneMinutePattern = 0;                          // flag indicating if one minute has passed
float amountOfSpeedsRecorded = 0;                   // variable containing amount of speeds recorded, used in calculation of average
float amountOfSpeedsRecordedOverSeventyPercent = 0; // variable containing amount of speeds recorded, used in calculation of time at or higher than70% speed
float averageSpeed;                                 // variable containing averageSpeed
int timeAtHighSpeeds;                               // variable containing time at speeds at or higher than 70%
float lastMinutesAverageSpeed = 0;                  // variable containing the last minutes average speed
float lastMinutesHighestSpeed = 0;                  // variable containing the last minutes Highest speed
int lastMinutestimeAtHighSpeeds = 0;                // variable containing the last minutes time at speeds at or higher than 70%
float totalOfSpeedsRecorded;                        // sum of speeds used in calculation of average
float highestSpeed = 0;                             // current highest speed recorded
float amphere = 1200;                               // value containing amphere hours left on battery
int percentOfBatteryLeft;                           // value containing amphere hours left on battery in percent
float batteryhealth = 100;                          // healthstatus of battery
float chargingcycles = 0;                           // amount of times battery has been charged
float timesFivePercentReached = 0;                  // amount of times battery has reached 5% charge
int batterylevel = 2;                               // batterylevel reperesenting batteryhealth
float average = 0;                                  // value containing getAverageSpeed
int placeInEepromWhereBatterhealtIsStored = 1;
/* byte location used to store value in EEPROM
worth noting that every EEPROM byte location only has 100k write/erase cycles
 so changing this value after multiple runs is not a bad idea, but with the current setup
 this code needs to run 11 days straight to exced the life cycle of a byte*/
int fivePercentFlag = 0;   // flag used to increase timesFivePercentReached
int emergencyCharging = 0; // flag used to enter emergencyCharging

// initiate serial and read value from EEPROM
void setup()
{
  Serial.begin(9600);
  display.init();
  encoders.init();
  int value = EEPROM.read(placeInEepromWhereBatterhealtIsStored);
  if (value != 255) // 255 is the inital value if an EEPROM byte has never been written to
  {
    batteryhealth = value;
  }
}

// function calculating distance traveled in a run
float getDistance(float speeds)
{
  unsigned long now = millis();
  float sInterval = speeds * (now / 1000 - distanceInterval / 1000); // s = v*t, unit is in counts
  distanceInterval = millis();
  sTotal = sTotal + sInterval;
  if (sTotal < 0)
  {
    sTotal = 0;
  }
  return sTotal;
}

int refreshRateTimesPerSec = 7; // times per second stepspersecond is updated
long stepCounter = 0;           // last loops steps
long stepsPerSecond = 0;        // steps per second
int totalCountsInSecond;        // total amount of steps in counts
unsigned long lastTime = 0;     // timing variable
int reversing = 0;              // flag for seeing if it is driving in reverse
float countsLeft;               // counts on the left encoder

// function used to get steps from encoders and check if reversed
long getStepsFromPoluluLib()
{
  countsLeft = encoders.getCountsAndResetLeft(); // gets counts
  if (countsLeft < 0)
  {
    reversing = 1;
    countsLeft = abs(countsLeft); // turns counts positve for positve distance calculation
  }
  totalCountsInSecond = totalCountsInSecond + countsLeft; // adds the amount of counts in a second

  return totalCountsInSecond;
}

void updateStepsPerSecond()
{
  long currentSteps = getStepsFromPoluluLib(); // gets current counts
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= 1000 / refreshRateTimesPerSec) // every 7th of a second
  {
    stepsPerSecond = (currentSteps - stepCounter) * refreshRateTimesPerSec; // steps per second
    stepCounter = currentSteps;                                             // previous 7th of a seconds counts
    totalCountsInSecond = 0;                                                // reset counts in second
    lastTime = millis();
  }
}

// function displaying speed and distance on the display, updating every other second
void displaySpeedAndDistanceInDisplay(float drive, float distance)
{

  if (timer.loopWait(2000)) // timer creates timed interval with a interval of 2 seconds
  {
    display.clear();
    display.print(drive);
    display.setCursor(0, 1);
    display.print(distance);
  }
}

// counts every second if speed is greater than 0
void count60secondinterval(float speeds)
{
  if (speeds > 0) // if the absolute value of speed calculated from earlier functions is not zero
  {

    unsigned long currentMinuteCounting = millis();
    if (currentMinuteCounting - previousOneMinuteCounting >= 60000)
    {
      oneMinutePattern = 1; // flag indicating that one second has passed
      previousOneMinuteCounting = millis();
    }
  }
}

// function that displays batterylevel, amount of times the battery has charged and batteryhealth on the LCD screen
void displayBatteryLevelAmountOfChargesBatteryHealth(int batteryLevel, int amountofcharges, float batteryHealth)
{

  unsigned long now = millis();
  if (now - displayrate >= 10000) // every 10th second
  {
    display.clear();
    display.print(batteryLevel);
    display.setCursor(7, 0); // sets position to maximize the area used on LCD
    display.print(amountofcharges);
    display.setCursor(2, 1);
    display.print(batteryHealth);
    displayrate = millis();
    EEPROM.update(placeInEepromWhereBatterhealtIsStored, batteryhealth);
  }
}

// function calculating average speed the last minute
float getAverageSpeed(float speeds)
{
  amountOfSpeedsRecorded++;                                      // counter keeping track of amount of speeds recorded
  totalOfSpeedsRecorded += speeds;                               // counter keeping sum of all speeds recorded
  averageSpeed = totalOfSpeedsRecorded / amountOfSpeedsRecorded; // avg = sum/amount
  if (oneMinutePattern == 1)                                     // if one minute passed
  {
    lastMinutesAverageSpeed = averageSpeed; // records current average speed
    // resets values
    totalOfSpeedsRecorded = 0;
    averageSpeed = 0;
  }
  return averageSpeed;
}

// function calculating maxspeed the last minute
float getMaxSpeed(float speeds)
{
  if (speeds > highestSpeed) // if loops speed is higher than the highest speed recorded
  {
    highestSpeed = speeds;
  }

  if (oneMinutePattern == 1) // if one minute has passed
  {
    lastMinutesHighestSpeed = highestSpeed; // records current highest speed
    highestSpeed = 0;                       // resets values
  }
  return highestSpeed;
}

// function calculating amount of speeds higher than 70 percent of max
float getspeedshigherthan70percent(float speeds)
{
  if (speeds >= 21) // after running max speed on the motors 21 was calculated as 70%
  {
    amountOfSpeedsRecordedOverSeventyPercent++; // counter keeping track of amount of highspeeds recorded
  }
  // amount of high speeds are the part, amount of speeds is max, and mapped between 0 to 60 to represent seconds
  timeAtHighSpeeds = map(amountOfSpeedsRecordedOverSeventyPercent, 0, amountOfSpeedsRecorded, 0, 60);
  if (oneMinutePattern == 1) // if one minute has passed
  {
    lastMinutestimeAtHighSpeeds = timeAtHighSpeeds; // records current time at high speeds
    // resets values
    amountOfSpeedsRecorded = 0;
    amountOfSpeedsRecordedOverSeventyPercent = 0;
    timeAtHighSpeeds = 0;
  }
  return timeAtHighSpeeds;
}

// function calculating the amount of charge left on battery, where charge is in amphere hours
float getAmphere(float averageSpeed, float Distance)
{
  unsigned long updateampheretimer = millis();
  if (updateampheretimer - changeAmphereRate >= 2000) // every other second
  {
    float y = (2 * averageSpeed + 1000) / 20;     // change based on average speed
    amphere = amphere * 3600;                     // multiplied at 3600 to convert to amphere
    amphere = amphere - (y + (Distance / 10000)); // decreases amphere to reflect used battery
    amphere = amphere / 3600;                     // converted back to amphere hours
    changeAmphereRate = millis();
  }
  return amphere;
} // end void getAmphere

// converts amphere to percents
int calculationOfBatteryLeft(float amphere)
{
  percentOfBatteryLeft = (amphere / 1200) * 100;
  return percentOfBatteryLeft;
}

// function that decreases batteryhealth
float batteryhealthchange(
    float chargincycles,
    float fivepercenttimes,
    float sixtysecaverage,
    float sixtysecspeed,
    float seventypercentofmax)
{
  float randomDefect = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 100)); // random float with range from 0.01 to 100.99
  int r;                                                                                  // flag indicating if defect has kicked inn
  if (randomDefect <= 0.1)                                                                // 0.01 chance of activating
  {
    r = 1;
  }
  else
  {
    r = 0;
  }
  unsigned long updatebatteryhealthtimer = millis();
  if (updatebatteryhealthtimer - changebatteryhealthrate >= 4000) // every 4th second
  {
    float y = (((chargincycles / 10 + fivepercenttimes / 10 + sixtysecaverage) * 100 + 1) / (100 + (sixtysecspeed * seventypercentofmax))) / 100;
    batteryhealth = batteryhealth - y - r * ((batteryhealth / 100) * 50); // r part reduces batteryhealth by 50 percent
    changebatteryhealthrate = millis();
  }

  if (batteryhealth <= 0) // negative get changed to zero
  {
    batteryhealth = 0;
  }
  return batteryhealth;
}

// change batterylevel based on batteryhealth
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

// function showing that shows that ten percent charge left
void tenPercentLeft()
{
  ledYellow(1); // yellow led turned on
  unsigned long current10perDisplayrate = millis();
  batterylevelhchange(batteryhealth);
  if (current10perDisplayrate - start10perDisplayrate >= 1000) // every second
  {
    display.clear();
    display.print("10 % ");
    buzzer.playFrequency(2000, 5, 7); // low pitch sound
    start10perDisplayrate = millis();
  }
}

// function showing that shows that five percent charge left
void fivePercentLeft()
{
  if (fivePercentFlag == 0) // flag used to keep count of times 5% charge reached
  {
    timesFivePercentReached++; // counter keeping track of 5 %
    fivePercentFlag = 1;
  }
  ledYellow(0);
  ledRed(1); // red led turned on
  unsigned long current5perDisplayrate = millis();
  batterylevelhchange(batteryhealth);
  if (current5perDisplayrate - start10perDisplayrate >= 15000) // every 15th second
  {
    buzzer.playFrequency(4000, 5, 10); // higher pitch sounds
    buzzer.playFrequency(4000, 5, 10);
    motors.setSpeeds(0, 0); // stops car
    display.clear();
    display.print("5 % ");
    start10perDisplayrate = millis();
  }
}

/* function that runs in default where it first takes in speed,
calculates average, max and 70% of max speed, calculates charge
calculates batteryhealth and display values on the LCD screen
*/
void normalDriving()
{
  encoders.getCountsAndResetLeft(); // resets steps before calculations of steps
  updateStepsPerSecond();
  float drive = abs(stepsPerSecond); // speed used for calculations
  count60secondinterval(drive);
  float distance = getDistance(drive); // distance
  if (drive > 0)                       // if speed not zero
  {
    // speed calculations
    average = getAverageSpeed(drive);
    float maxSpeed = getMaxSpeed(drive);
    float seventyper = getspeedshigherthan70percent(drive);
  }
  // charge calculations
  getAmphere(average, distance);
  calculationOfBatteryLeft(amphere);
  if (percentOfBatteryLeft <= 10 && percentOfBatteryLeft > 5) // percents between 10& and 5%
  {
    tenPercentLeft();
  }
  if (percentOfBatteryLeft <= 5) // percents under 5%
  {
    fivePercentLeft();
  }
  // batteryhealth functions
  batteryhealthchange(chargingcycles, timesFivePercentReached, lastMinutesAverageSpeed, lastMinutesHighestSpeed, lastMinutestimeAtHighSpeeds);
  batterylevelhchange(batteryhealth);
  // display functions
  displaySpeedAndDistanceInDisplay(drive, distance);
  displayBatteryLevelAmountOfChargesBatteryHealth(batterylevel, chargingcycles, batteryhealth);
  oneMinutePattern = 0;
}

// same numbers as amphere but adds instead of subracts
float getPositiveAmphere(float averageSpeed, float Distance)
{

  unsigned long updateampheretimer = millis();
  if (updateampheretimer - changePositiveAmphereRate >= 10)
  {
    float y = (2 * averageSpeed + 10) / 200;
    amphere = amphere * 3600;
    amphere = amphere + (y + (Distance / 5000));
    amphere = amphere / 3600;
    if (amphere > 1200)
    {
      amphere = 1200;
    }
    changePositiveAmphereRate = millis();
  }
  return amphere;
} // end void getAmphere

/*
function with the same structure as normal driving, but with a
added if structur that charges if the car is driving in reverse
*/
void charginInReverse()
{
  encoders.getCountsAndResetLeft();
  updateStepsPerSecond();
  float drive = abs(stepsPerSecond);
  count60secondinterval(drive);
  float distance = getDistance(drive);
  if (drive > 0)
  {
    average = getAverageSpeed(drive);
    float maxSped = getMaxSpeed(drive);
    float seventyPer = getspeedshigherthan70percent(drive);
  }

  if (buttonC.isPressed() && emergencyCharging == 0) // emergency charging mode
  {
    if (reversing == 0) // if reversing false
    {
      getAmphere(average, distance);
      reversing = 0;
    }
    if (reversing == 1) // if reversing true
    {
      getPositiveAmphere(10 * average, distance); // 10 times amount of charge
      reversing = 0;
    }
    if (percentOfBatteryLeft > 20) // if more than 20% of charge
    {
      emergencyCharging = 1; // flag set to one so that emergency charge only can be entered once
    }
  }

  if (reversing == 0)
  {
    getAmphere(average, distance); // discharges battery
    reversing = 0;
  }
  if (reversing == 1)
  {
    getPositiveAmphere(average, distance); // charges battery
    reversing = 0;
  }
  calculationOfBatteryLeft(amphere); // percents left
  // batteryhealth functions
  batteryhealthchange(chargingcycles, timesFivePercentReached, lastMinutesAverageSpeed, lastMinutesHighestSpeed, lastMinutestimeAtHighSpeeds);
  batterylevelhchange(batteryhealth);
  // display functions
  displaySpeedAndDistanceInDisplay(drive, distance);
  displayBatteryLevelAmountOfChargesBatteryHealth(batterylevel, chargingcycles, batteryhealth);
  oneMinutePattern = 0;
}

/*
Function where car stops and goes into charging mode and
can have battery service or change battery
*/
void chargingMode()
{
  motors.setSpeeds(0, 0); // stops car
  // turns of leds
  ledRed(0);
  ledYellow(0);
  display.clear();
  // resets flags
  fivePercentFlag = 0;
  int flag = 1;
  int cleanenergy = 1; // if charging on clean energy
  int costCharging;
  int amountOfMoneyUsedInCharge = 0; // money used on charging and batter service

  while (account_balance >= 0 && amphere < 1200) // while not full charge and money in account
  {
    if (buttonA.getSingleDebouncedPress()) // regular charging
    {
      if (flag == 1) // flag to only add one to chargingcycles every charging mode
      {
        chargingcycles++;
        flag = 0;
      }

      if (cleanenergy == 1) // if clean energy
      {
        costCharging = 25;
      }

      if (cleanenergy != 1) // if not clean energy
      {
        costCharging = 50;
      }
      amphere = amphere + 120;
      account_balance = account_balance - costCharging;                     // takes money from account
      amountOfMoneyUsedInCharge = amountOfMoneyUsedInCharge + costCharging; // increases money used in charge
      if (amphere > 1200)                                                   // To not go over max charge
      {
        amphere = 1200;
      }
      ledYellow(1); // turns on led to indicate that money has been used
    }

    if (buttonB.getSingleDebouncedPress()) // battery service
    {
      batteryhealth = batteryhealth + 15;
      if (batteryhealth >= 100)
      {
        batteryhealth = 100;
      }
      EEPROM.update(placeInEepromWhereBatterhealtIsStored, batteryhealth); // updates value in EEPROM
      account_balance = account_balance - 200;
      amountOfMoneyUsedInCharge = amountOfMoneyUsedInCharge + 200;
    }
    if (buttonC.getSingleDebouncedPress()) // battery change
    {
      // resets values
      batteryhealth = 100;
      sTotal = 0;
      EEPROM.update(placeInEepromWhereBatterhealtIsStored, batteryhealth);
      // monet
      account_balance = account_balance - 500;
      amountOfMoneyUsedInCharge = amountOfMoneyUsedInCharge + 500;
    }
    unsigned long currentChargingDisplayrate = millis();
    batterylevelhchange(batteryhealth);
    if (currentChargingDisplayrate - startChargingDisplayrate >= 500) // every half second
    {
      // displays values on LCD screen
      display.clear();
      display.print(account_balance);
      display.setCursor(5, 0);
      display.print(amountOfMoneyUsedInCharge);
      display.setCursor(0, 1);
      display.print(batterylevel);
      display.setCursor(4, 1);
      display.print(amphere);
      startChargingDisplayrate = millis();
    }
  }
  ledYellow(0); // turns off led to show that charging is complete
}

// function indicating that batteryhealth or charge is zero
void deadbattery()
{
  motors.setSpeeds(0, 0);
  display.clear();
  display.print("Your out of power");
  display.setCursor(0, 1);
  display.print("Press A");
  buttonA.waitForButton(); // stand still until button is pressed
  // resets values
  batteryhealth = 100;
  amphere = 1200;
  batterylevel = 2;
  sTotal = 0;
  EEPROM.update(placeInEepromWhereBatterhealtIsStored, batteryhealth);
  batterylevelhchange(batteryhealth);
  account_balance = account_balance - 2000; // expensive to get towed
}

void loop()
{
  // state used to decide case
  int state = 0;
  if (buttonA.isPressed())
  {
    state = 1; // case 1
  }
  if (buttonB.isPressed())
  {
    state = 2; // case 2
  }

  if (batterylevel <= 0 || amphere <= 0) // if dead battery
  {
    amphere = 0;
    batterylevel = 0;
    state = 3;
  }

  switch (state)
  {

  case 1: // charging mode
  {
    chargingMode();
  }
  break;

  case 2: // reverse charging
  {
    charginInReverse();
  }
  break;

  case 3: // dead battery
    deadbattery();
    break;

  default: // driving without anything
  {
    normalDriving();
  }
  break;
  } // end switch case
} // end void loop
