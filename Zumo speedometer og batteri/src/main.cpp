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
// last error variabel
int last_error = 0;

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

// venstre: 4000, høyre: 0

void pd_regulator()
{
  int wanted_value = 2000;
  int topSpeed = 400;
  const int posisjon = linesensor.readLine(lineSensorValues);

  int kp = 1;
  int error = posisjon - wanted_value;
  int d_ledd = 2 * (error - last_error);
  int speedDifference = error * kp + d_ledd;
  int leftSpeed = 400 - speedDifference;
  int rightSpeed = 400 + speedDifference;
  // bruker constrain så verdiene holder seg mellom 0 og 400
  leftSpeed = constrain(leftSpeed, 0, topSpeed);
  rightSpeed = constrain(rightSpeed, 0, topSpeed);
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
  // hvis oppgavestatus er OPPGAVE3 -> kjør oppgave3()
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