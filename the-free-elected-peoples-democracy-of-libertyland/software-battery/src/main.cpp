#include <Arduino.h>
#include <Wire.h>
#include <Zumo32U4.h>
#include <PololuMenu.h>
#include <Vector.h>
#include <Zumo32U4Encoders.h>
#include <Zumo32U4Motors.h>

//til testing av switch case og funksjoner 

Zumo32U4Buzzer buzzer;
Zumo32U4LCD display;
Zumo32U4Encoders encoder;
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;

float amphere = 1200;//current value in battery 
//unsigned longs used for timing purposes
unsigned long wait = 0;
unsigned long interval = 0;
unsigned long count = 0;
unsigned long pattern = 0;
unsigned long startMillis = 0;
unsigned long n = 0;
unsigned long displayrate = millis();
//returned values
float highestSpeed = 0;//highestspeed in 60 sec interval
float vel = 0;//value returning speed calculation
float sTotal = 0;//value returning total distance
float timeAtHighSpeeds;//value returning amount of time at 70% or higher speed
float averageSpeed = 0;//value returning averagespeed in 60sec interval
float seventypercentofmaxspeed = 70;//value defining the 70% mark
int percentofBatteryleft = 0;//amount of percent left on battery
int reversecharge = 20;//charge in reverse charging mode
float batteryhealth;//current battery health
int amountofchargingcycles = 0;//amount of times battery has charged
int timesreachedfivepercentcharge = 0;//amount of times battery has reached five percent
const int ELEMENT_COUNT_MAX = 1;//value used to creat vector
typedef Vector<int> Elements;//define vector
int storage_array[ELEMENT_COUNT_MAX];//values used to set storage
Elements vector;//vector used to keep values
Elements vector2;//vector2 used to keep values

const char tenpercentsound[] PROGMEM = 
"!af";

const char fivepercentsound[] PROGMEM = 
"!af";

 void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  randomSeed((unsigned int) millis());
  display.init();
  vector.setStorage(storage_array);
  vector2.setStorage(storage_array);
}

float getSpeed(int refreshRate = 10) {
  int startCounts = encoder.getCountsRight();
  unsigned long now = millis();
  while(millis()< now + refreshRate){}
  int counts = encoder. getCountsAndResetRight() - startCounts;
  float cm = counts / 195.0;
  vel = (cm / refreshRate) * 1000;
  return vel;
}//end float get speed

float getDistance(float speed){
    unsigned long now=millis(); 
    float sInterval = speed*(now/1000 - interval/1000);
    interval=millis();
    sTotal = sTotal + sInterval;
    return sTotal;
}



void popVectors(){
    for (unsigned int i = vector.size(); i == 0; i-- ){
      vector.pop_back();
    }
    for (unsigned int i = vector2.size(); i == 0; i-- ){
      vector2.pop_back();
    }

  }

void count60secondinterval(){
  int startCounts = encoder.getCountsRight();
  if (startCounts != 0){
    unsigned long now=millis(); 
    int timeinterval = (now/1000)-(count/1000);
    count = millis();
    pattern = pattern + timeinterval;
}
}


float getMaxSpeed (float x)
{
  if (x > highestSpeed){
   highestSpeed = x;
   
  }
  return highestSpeed;
}

float getAverageSpeed(float y)
{
  vector.push_back(y);
  if (pattern >=60){
    for (unsigned int i = 0; i >= vector.size(); i++ ){
    n = n + vector.at(i);
    averageSpeed = n/vector.size();
    }
    return averageSpeed;
  }
  else{
    return averageSpeed;
  }
}

float getAmphere(float getSpeed){
  float y = 2*getAverageSpeed(getSpeed)+10; 
  amphere = amphere*3600;
  amphere = amphere-y*(getDistance(getSpeed)/1000); 
  amphere = amphere/3600;
  return amphere;
}//end void getAmphere

float getspeedshigherthan70percent()
{
  if (pattern >= 60){
    for (unsigned int i = 0; i >= vector.size(); i++ ){
      if (vector.at(i) >= seventypercentofmaxspeed){
      vector2.push_back(vector.at(i));
      }
    }
    timeAtHighSpeeds = (60*((vector2.size()*100)/vector.size()))/100;

    return timeAtHighSpeeds;
  }

  else{
    return timeAtHighSpeeds;
  }
}

void resultsOf60SecondInterval(){
     count60secondinterval();
     if (pattern >= 60){
     getMaxSpeed(getSpeed(10));
     getAverageSpeed(getSpeed(10));
     getspeedshigherthan70percent();
     pattern=0;
     popVectors();
     display.clear();
     display.print(highestSpeed);
     display.setCursor(0, 1);
     display.print(averageSpeed);
     display.setCursor(0, 2);
     display.print(timeAtHighSpeeds);
     unsigned long now = millis();
     while (millis() <= now + 500){}
    }
}

int percentOfBatteryLeft(float amphere){
  percentofBatteryleft=amphere/1200*100;
  return percentofBatteryleft;
}

void chargingmode(){

}

void hiddencharging(){

}

void emergencycharge(){

}

float batteryhealthchange(
  float chargincycles, 
  float fivepercenttimes, 
  float sixtysecaverage,
  float sixtysecspeed,
  float seventypercentofmax)
   {
  int malfunction = random(0,100);

  if (malfunction <= 5){
    batteryhealth = batteryhealth - ((chargincycles*2+fivepercenttimes*4+sixtysecaverage)/15)+((sixtysecspeed*seventypercentofmax)/150)-((batteryhealth*50)/100);
    return batteryhealth;
  }
  else{
  batteryhealth = batteryhealth - ((chargincycles*2+fivepercenttimes*4+sixtysecaverage)/15)+((sixtysecspeed*seventypercentofmax)/150);
  return batteryhealth; 
  }
}

void tenpercentbatteryleft(){
unsigned long now = millis();
display.clear();//clearer displayet
display.setCursor(0,0);//setter linja som skriver i displayet til koordinat 0x,0y
display.print("10 percent left");//printer beskjeden i displayet
while (millis() <= now + 4000){
buzzer.playFromProgramSpace(tenpercentsound);
while(buzzer.isPlaying()){
  ledYellow(1);
  now = millis();
  while (millis() <= now + 500){}
  ledYellow(0);
}
buzzer.stopPlaying();
ledYellow(0);
}
}

void fivepercentleft(){
unsigned long now = millis();
display.clear();//clearer displayet
display.setCursor(0,0);//setter linja som skriver i displayet til koordinat 0x,0y
display.print("5 percent left");//printer beskjeden i displayet
buzzer.playFromProgramSpace(fivepercentsound);
  ledRed(1);
  now = millis();
  while (millis() <= now + 500){}
  ledRed(0);
  unsigned long currentMillis = millis();
  if (currentMillis - startMillis >= 15000)
  {
    motors.setRightSpeed(0);
    motors.setLeftSpeed(0);
    buzzer.stopPlaying();
    buzzer.playFrequency(6000,100,15);
    buzzer.playFrequency(6000,100,15);
  }
  startMillis = millis();
}

void loop(){
  float r2 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/15));
  Serial.println(r2);
  getDistance(r2);
  Serial.println(sTotal);
  unsigned long now = millis();
  while (millis() <= now + 500){}


  // put your main code here, to run repeatedly:
motors.setSpeeds(50,50);
int state=0;
if(buttonA.getSingleDebouncedPress()){
state = 1;
}
if(buttonB.getSingleDebouncedPress()){
state = 2;
}

else{
  state=0;
}
if(batteryhealth <= 0 || amphere <= 0){
amphere = 0;
batteryhealth = 0;
state = 4;
}

switch (state)
{

case 1://charging mode
{
display.clear();
display.print("Your in case 1");
unsigned long show = millis();
while (millis() <= show + 1000){}

} 
break;

case 2://reverse charging
{
display.clear();
display.print("Your in case 2");
unsigned long snow = millis();
while (millis() <= snow + 1000){}
}
break;

case 3://dead battery
break;

default://driving without anything
{
  count60secondinterval();
  float drive = getSpeed(10);
  float distance = getDistance(drive);
  display.clear();
  display.print(drive);
  display.setCursor(0, 1);
  display.print(distance);
  float battery = getAmphere(drive);
  int percents = percentOfBatteryLeft(battery);
  if (percents <= 10 && percents > 5){
    tenpercentbatteryleft();
  }
  if (percents <= 5){
    fivepercentleft();
  }
  resultsOf60SecondInterval();
  batteryhealthchange(amountofchargingcycles,timesreachedfivepercentcharge,averageSpeed,highestSpeed,seventypercentofmaxspeed);
  unsigned long now = millis();
  if(now - displayrate >= 10000){
    display.clear();
    display.print(percents);
    display.setCursor(0, 1);
    display.print(amountofchargingcycles);
    display.setCursor(0, 2);
    display.print(batteryhealth);
    while (millis() <= now + 1000){}
  }
  buzzer.playFrequency(6000,1000,10);
}
  break;
}//end switch case
}//end void loop






