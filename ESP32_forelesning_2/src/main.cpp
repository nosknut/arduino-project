//#include <WiFi.h>
//#include <WebServer.h>
#include <Arduino.h>

// for å velge oppgave bruker jeg enum class:

enum class OppgaveStatus
{
  OPPGAVE3_A,
  OPPGAVE3_B
};
// her velger man oppgave:
OppgaveStatus oppgave = OppgaveStatus::OPPGAVE3_A;

// definerer pinner til sensorer:
#define PHOTO_PIN 33
#define POT_PIN 34
#define TEMP_PIN 35

void setup()
{
  Serial.begin(9600);
}

// finner spenning for valgt pin
float getVoltageFor(int pin)
{
  int value = analogRead(pin);
  return (value * 3.3) / 4095;
}

// finner temp for valgt pin (fungerer for tmp36gz)
float getTemperatureCelsiusFor(int pin)
{
  float voltage = getVoltageFor(pin);
  return (voltage - 0.5) * 100;
}

void loop()
{
  if (oppgave == OppgaveStatus::OPPGAVE3_A)
  {
    // henter potensometer verdi og gjør om til 0 -> 10 000
    int potSensorReading = analogRead(POT_PIN);
    potSensorReading = map(potSensorReading, 0, 4096, 0, 10000);
    // henter spenningsverdi temp sensor
    float tempSensorReading = getTemperatureCelsiusFor(TEMP_PIN);
    // henter verdi for photoresistor
    int photoResistorReading = analogRead(PHOTO_PIN);

    // printer til Serial:
    Serial.print("POT_METER: ");
    Serial.print(potSensorReading);
    Serial.print("TEMP: ");
    Serial.print(tempSensorReading);
    Serial.print("PHOTO_RES: ");
    Serial.print(photoResistorReading);
  }
}

/*// han skrev egt: const char* ssid = "noe";
const char *ssid = "Kristians iphone ellerno"; // navn på wifi lokal
const char *password = "123456";

WebServer server(80);

const int sensorPin1 = 34;
const int sensorPin2 = 35;

unsigned long previousMillis = 0;

// designer nettsiden:
String refreshData()
{
  float sensorDataArr[2]; // array - tom
  sensorDataArr[0] = analogRead(sensorPin1);
  sensorDataArr[1] = analogRead(sensorPin2);

  Serial.println(sensorDataArr[0]);
  Serial.println(sensorDataArr[1]);

  String HTMLdata = "<!DOCTYPE html>"
                    "<html>"
                    "<head>"
                    "<title>Sensordata</title>"
                    "<meta http-equiv='refresh' content='0.5'>"
                    "</head>"
                    "<body>"
                    "<h1>Sensordata:</h1>"
                    "</body>"
                    "</html>";

  HTMLdata += "<h2>Sensor 1: ";
  HTMLdata += sensorDataArr[0];
  HTMLdata += "</h2>";

  HTMLdata += "<h2>Sensor 2: ";
  HTMLdata += sensorDataArr[1];
  HTMLdata += "</h2>";

  return HTMLdata;
}

void handle_root()
{
  server.send(200, "text/html", refreshWebData());
}

void setup()
{
  Serial.begin(9600);
  analogReadResolution(10);
  Serial.println()
}

void loop()
{
}*/