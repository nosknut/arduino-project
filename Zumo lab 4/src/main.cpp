#include <Arduino.h>
#include <Zumo32U4.h>
#include <Zumo32U4Buttons.h>

Zumo32U4ButtonA ButtonA; // knapp a
Zumo32U4ButtonB ButtonB; // knapp b
Zumo32U4ButtonC ButtonC; // knapp c

// buzzer variabel
Zumo32U4Buzzer buzzer;
// motor variabel
Zumo32U4Motors motors;
// line sensor
Zumo32U4LineSensors linesensor;
const int NUM_SENSORS = 5;
unsigned int lineSensorValues[NUM_SENSORS];
// lcd variabel
Zumo32U4LCD lcd;

// bruker enum til å velge hvilken oppgave som skal kjøres
enum class OppgaveStatus
{
  OPPGAVE1,
  OPPGAVE2,
  OPPGAVE3,
  OPPGAVE4
};

//-----VELG HVILKEN OPPGVAVE SOM SKAL KJØRES------
OppgaveStatus oppgave = OppgaveStatus::OPPGAVE4;

void kalibrerSensor()
{
  lcd.clear();
  lcd.gotoXY(0, 0);
  lcd.print("Trykk A");
  lcd.gotoXY(0, 1);
  lcd.print("for kalibrering");
  // venter på knappetrykk
  // fungerer som en delay
  ButtonA.waitForButton();
  long time = millis() + 4000;

  while (time > millis())
  {
    motors.setSpeeds(200, -200);
    linesensor.calibrate();
  }

  motors.setSpeeds(0, 0);
  lcd.clear();
  lcd.gotoXY(0, 0);
  lcd.print("Kalibrering ferdig");
  delay(2000);
}

void setup()
{
  if (oppgave == OppgaveStatus::OPPGAVE3 || oppgave == OppgaveStatus::OPPGAVE4)
  {
    linesensor.initFiveSensors();
    // hvis du tester oppgave 3 eller 4; kjør:
    kalibrerSensor();
  }
}

void oppgave1()
{
  if (ButtonA.isPressed()) // hvis knapp er trykt
  {
    // blinker alle led 4 ganger med delay:
    for (int i = 0; i < 4; i++)
    {
      ledRed(HIGH);
      ledGreen(HIGH);
      ledYellow(HIGH);
      delay(100);
      ledRed(LOW);
      ledGreen(LOW);
      ledYellow(LOW);
      delay(100);
    }
    // Starter med frekvens 440 Hz og volum 15
    buzzer.playFrequency(440, 200, 15);
    delay(1000); // litt delay så den blir ferdig

    // Spiller note A med volum 15
    buzzer.playNote(NOTE_A(4), 2000, 15);
    delay(200);           // venter til noten er ferdig
    buzzer.stopPlaying(); // stopper buzzer
  }
}

// tar inn hvilken side (false = left, true = right)
// speed from -> to og time (til delay)
// ser nå at dette kan gjøres med funksjon "setSpeeds()"
void motorRun(bool side, int from, int to, int time)
{
  // høyre side
  if (side)
  {
    for (int speed = from; speed <= to; speed++)
    {
      motors.setRightSpeed(speed);
      delay(time);
    }
  }
  // venstre side
  else
  {
    for (int speed = from; speed <= to; speed++)
    {
      motors.setLeftSpeed(speed);
      delay(time);
    }
  }
}

// mønster 1
void task1(int time)
{
  for (int i = 0; i < 2; i++)
  {
    motors.setSpeeds(200, 50);
    delay(time);

    motors.setSpeeds(50, 200);
    delay(time);
  }
}
// mønster 2
void task2(int time)
{
  motors.setSpeeds(0, 200);
  delay(time);
}
// mønster 3
void task3(int time)
{
  motors.setSpeeds(200, 0);
  delay(time);
}

// venstre: 4000, høyre: 0

void oppgave4(const int sensorData)
{
  int right; // høyre motor fart
  int left;  // venstre motor fart
  // har en liten "feilmargin på 100"
  if (sensorData >= 2050)
  {
    // bruker map funksjon for å sette hastighet til motor
    left = map(sensorData, 2050, 4000, 100, 150);
    right = map(sensorData, 2050, 4000, 100, 0);
  }
  if (sensorData <= 1950)
  {
    left = map(sensorData, 1950, 0, 100, 0);
    right = map(sensorData, 1950, 0, 100, 150);
  }
  // hvis vi er innenfor "feilmargin" sett fart:
  else
  {
    right, left = 100;
  }
  // oppdaterer motor fart
  motors.setSpeeds(left, right);
}

void loop()
{
  if (oppgave == OppgaveStatus::OPPGAVE1)
  {
    oppgave1();
  }
  if (oppgave == OppgaveStatus::OPPGAVE2)
  {
    task1(3000);
    task2(4000);
    task3(4000);
  }
  if (oppgave == OppgaveStatus::OPPGAVE3)
  {
    const int posisjon = linesensor.readLine(lineSensorValues);
    lcd.clear();
    lcd.gotoXY(0, 0);
    lcd.print("Posisjon: ");
    lcd.gotoXY(0, 1);
    lcd.print(posisjon);
    delay(20);
  }
  if (oppgave == OppgaveStatus::OPPGAVE4)
  {
    const int posisjon = linesensor.readLine(lineSensorValues);
    lcd.clear();
    lcd.gotoXY(0, 0);
    lcd.print("Posisjon: ");
    lcd.gotoXY(0, 1);
    lcd.print(posisjon);
    delay(20);
    oppgave4(posisjon);
  }
}