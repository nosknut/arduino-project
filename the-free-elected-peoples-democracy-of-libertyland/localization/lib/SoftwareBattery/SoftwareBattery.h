#ifndef SoftwareBattery_h
#define SoftwareBattery_h

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
int placeInEepromWhereBatterhealtIsStored = 1;      // byte location used to store value in EEPROM
int fivePercentFlag = 0;                            // flag used to increase timesFivePercentReached
int emergencyCharging = 0;                          // flag used to enter emergencyCharging

void setup()
{
    Serial.begin(9600);
    display.init();
    encoders.init();
    int value = EEPROM.read(placeInEepromWhereBatterhealtIsStored);
    if (value != 255)
    {
        batteryhealth = value;
    }
}

float getDistance(float speeds)
{
    unsigned long now = millis();
    float sInterval = speeds * (now / 1000 - distanceInterval / 1000);
    distanceInterval = millis();
    sTotal = sTotal + sInterval;
    if (sTotal < 0)
    {
        sTotal = 0;
    }
    return sTotal;
}

int refreshRateTimesPerSec = 7;
long stepCounter = 0;
long stepsPerSecond = 0;
int totalCountsInSecond;
unsigned long lastTime = 0;
int reversing = 0;
float countsLeft;

long getStepsFromPoluluLib()
{
    countsLeft = encoders.getCountsAndResetLeft();
    if (countsLeft < 0)
    {
        reversing = 1;
        countsLeft = abs(countsLeft);
    }
    totalCountsInSecond = totalCountsInSecond + countsLeft;

    return totalCountsInSecond;
}

void updateStepsPerSecond()
{
    long currentSteps = getStepsFromPoluluLib();
    unsigned long currentTime = millis();
    if (currentTime - lastTime >= 1000 / refreshRateTimesPerSec)
    {
        stepsPerSecond = (currentSteps - stepCounter) * refreshRateTimesPerSec;
        stepCounter = currentSteps;
        totalCountsInSecond = 0;
        lastTime = millis();
    }
}

void displaySpeedAndDistanceInDisplay(float drive, float distance)
{

    if (timer.loopWait(2000))
    {
        display.clear();
        display.print(drive);
        display.setCursor(0, 1);
        display.print(distance);
    }
}

unsigned long countStart;

void count60secondinterval(float speeds)
{
    if (speeds > 0)
    {

        unsigned long currentMinuteCounting = millis();
        if (currentMinuteCounting - previousOneMinuteCounting >= 60000)
        {
            oneMinutePattern = 1;
            previousOneMinuteCounting = millis();
        }
    }
}

void displayBatteryLevelAmountOfChargesBatteryHealth(int batteryLevel, int amountofcharges, float batteryHealth)
{

    unsigned long now = millis();
    if (now - displayrate >= 10000)
    {
        display.clear();
        display.print(batteryLevel);
        display.setCursor(7, 0);
        display.print(amountofcharges);
        display.setCursor(2, 1);
        display.print(batteryHealth);
        displayrate = millis();
        EEPROM.update(placeInEepromWhereBatterhealtIsStored, batteryhealth);
        float value = EEPROM.read(placeInEepromWhereBatterhealtIsStored);
    }
}

float getAverageSpeed(float speeds)
{
    amountOfSpeedsRecorded++;
    totalOfSpeedsRecorded += speeds;
    averageSpeed = totalOfSpeedsRecorded / amountOfSpeedsRecorded;
    if (oneMinutePattern == 1)
    {
        lastMinutesAverageSpeed = averageSpeed;
        totalOfSpeedsRecorded = 0;
        averageSpeed = 0;
    }
    return averageSpeed;
}

float getMaxSpeed(float speeds)
{
    if (speeds > highestSpeed)
    {
        highestSpeed = speeds;
    }

    if (oneMinutePattern == 1)
    {
        lastMinutesHighestSpeed = highestSpeed;
        highestSpeed = 0;
    }
    return highestSpeed;
}

float getspeedshigherthan70percent(float speeds)
{
    if (speeds >= 21)
    {
        amountOfSpeedsRecordedOverSeventyPercent++;
    }
    timeAtHighSpeeds = map(amountOfSpeedsRecordedOverSeventyPercent, 0, amountOfSpeedsRecorded, 0, 60);
    if (oneMinutePattern == 1)
    {
        lastMinutestimeAtHighSpeeds = timeAtHighSpeeds;
        amountOfSpeedsRecorded = 0;
        amountOfSpeedsRecordedOverSeventyPercent = 0;
        timeAtHighSpeeds = 0;
    }
    return timeAtHighSpeeds;
}

float getAmphere(float averageSpeed, float Distance)
{
    unsigned long updateampheretimer = millis();
    if (updateampheretimer - changeAmphereRate >= 2000)
    {
        float y = (2 * averageSpeed + 1000) / 20;
        amphere = amphere * 3600;
        amphere = amphere - (y + (Distance / 10000));
        amphere = amphere / 3600;
        changeAmphereRate = millis();
    }
    return amphere;
} // end void getAmphere

int calculationOfBatteryLeft(float amphere)
{
    percentOfBatteryLeft = (amphere / 1200) * 100;
    return percentOfBatteryLeft;
}

float batteryhealthchange(
    float chargincycles,
    float fivepercenttimes,
    float sixtysecaverage,
    float sixtysecspeed,
    float seventypercentofmax)
{
    float randomDefect = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 100));
    Serial.println(randomDefect);
    int r;
    if (randomDefect <= 0.1)
    {
        r = 1;
    }
    else
    {
        r = 0;
    }
    unsigned long updatebatteryhealthtimer = millis();
    if (updatebatteryhealthtimer - changebatteryhealthrate >= 4000)
    {
        float y = (((chargincycles / 10 + fivepercenttimes / 10 + sixtysecaverage) * 100 + 1) / (100 + (sixtysecspeed * seventypercentofmax))) / 100;
        batteryhealth = batteryhealth - y - r * ((batteryhealth / 100) * 50);
        changebatteryhealthrate = millis();
    }
    // TODO random defect reducing battery by fifty
    if (batteryhealth <= 0)
    {
        batteryhealth = 0;
    }
    return batteryhealth;
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

void tenPercentLeft()
{
    ledYellow(1);
    unsigned long current10perDisplayrate = millis();
    batterylevelhchange(batteryhealth);
    if (current10perDisplayrate - start10perDisplayrate >= 1000)
    {
        display.clear();
        display.print("10 % ");
        buzzer.playFrequency(2000, 5, 7);
        start10perDisplayrate = millis();
    }
}

void fivePercentLeft()
{
    if (fivePercentFlag == 0)
    {
        timesFivePercentReached++;
        fivePercentFlag = 1;
    }
    ledYellow(0);
    ledRed(1);
    unsigned long current5perDisplayrate = millis();
    batterylevelhchange(batteryhealth);
    if (current5perDisplayrate - start10perDisplayrate >= 15000)
    {
        buzzer.playFrequency(4000, 5, 10);
        buzzer.playFrequency(4000, 5, 10);
        motors.setSpeeds(0, 0);
        display.clear();
        display.print("5 % ");
        start10perDisplayrate = millis();
    }
}

void normalDriving()
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
        float seventyper = getspeedshigherthan70percent(drive);
    }
    getAmphere(average, distance);
    calculationOfBatteryLeft(amphere);
    if (percentOfBatteryLeft <= 10 && percentOfBatteryLeft > 5)
    {
        tenPercentLeft();
    }
    if (percentOfBatteryLeft <= 5)
    {
        fivePercentLeft();
    }
    batteryhealthchange(chargingcycles, timesFivePercentReached, lastMinutesAverageSpeed, lastMinutesHighestSpeed, lastMinutestimeAtHighSpeeds);
    batterylevelhchange(batteryhealth);
    displaySpeedAndDistanceInDisplay(drive, distance);
    displayBatteryLevelAmountOfChargesBatteryHealth(batterylevel, chargingcycles, batteryhealth);
    oneMinutePattern = 0;
}

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

    if (buttonC.isPressed() && emergencyCharging == 0)
    {
        if (reversing == 0)
        {
            getAmphere(average, distance);
            reversing = 0;
        }
        if (reversing == 1)
        {
            getPositiveAmphere(10 * average, distance);
            reversing = 0;
        }
        if (percentOfBatteryLeft > 20)
        {
            emergencyCharging = 1;
        }
    }

    if (reversing == 0)
    {
        getAmphere(average, distance);
        reversing = 0;
    }
    if (reversing == 1)
    {
        getPositiveAmphere(average, distance);
        reversing = 0;
    }
    calculationOfBatteryLeft(amphere);
    batteryhealthchange(chargingcycles, timesFivePercentReached, lastMinutesAverageSpeed, lastMinutesHighestSpeed, lastMinutestimeAtHighSpeeds);
    batterylevelhchange(batteryhealth);
    displaySpeedAndDistanceInDisplay(drive, distance);
    displayBatteryLevelAmountOfChargesBatteryHealth(batterylevel, chargingcycles, batteryhealth);
    oneMinutePattern = 0;
}

void chargingMode()
{
    motors.setSpeeds(0, 0);
    ledRed(0);
    ledYellow(0);
    display.clear();
    fivePercentFlag = 0;
    int flag = 1;
    int cleanenergy = 1;
    int costCharging;
    int amountOfMoneyUsedInCharge = 0;

    while (account_balance >= 0 && amphere < 1200)
    {
        if (buttonA.getSingleDebouncedPress()) // regular charging
        {
            if (flag == 1)
            {
                chargingcycles++;
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
            batteryhealth = batteryhealth + 15;
            if (batteryhealth >= 100)
            {
                batteryhealth = 100;
            }
            EEPROM.update(placeInEepromWhereBatterhealtIsStored, batteryhealth);
            account_balance = account_balance - 200;
            amountOfMoneyUsedInCharge = amountOfMoneyUsedInCharge + 200;
        }
        if (buttonC.getSingleDebouncedPress()) // battery change
        {
            batteryhealth = 100;
            sTotal = 0;
            EEPROM.update(placeInEepromWhereBatterhealtIsStored, batteryhealth);
            account_balance = account_balance - 500;
            amountOfMoneyUsedInCharge = amountOfMoneyUsedInCharge + 500;
            sTotal = 0;
        }
        unsigned long currentChargingDisplayrate = millis();
        batterylevelhchange(batteryhealth);
        if (currentChargingDisplayrate - startChargingDisplayrate >= 500)
        {
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
    ledYellow(0);
}

void deadbattery()
{
    motors.setSpeeds(0, 0);
    display.clear();
    display.print("Your out of power");
    display.setCursor(0, 1);
    display.print("Press A");
    buttonA.waitForButton();
    batteryhealth = 100;
    amphere = 1200;
    batterylevel = 2;
    sTotal = 0;
    EEPROM.update(placeInEepromWhereBatterhealtIsStored, batteryhealth);
    batterylevelhchange(batteryhealth);
    account_balance = account_balance - 2000; // thanks for the tow :)
}

void loop()
{
    motors.setSpeeds(100, 100);
    int state = 0;
    if (buttonA.isPressed())
    {
        state = 1;
    }
    if (buttonB.isPressed())
    {
        state = 2;
    }

    if (batterylevel <= 0 || amphere <= 0)
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
        encoders.getCountsAndResetLeft();
    }
    break;

    case 2: // reverse charging
    {
        charginInReverse();
    }
    break;

    case 3: // dead battery
        deadbattery();
        encoders.getCountsAndResetLeft();
        break;

    default: // driving without anything
    {
        normalDriving();
    }
    break;
    } // end switch case
} // end void loop

#endif
