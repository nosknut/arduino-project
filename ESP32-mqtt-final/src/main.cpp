#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

// Wifi navn (ssid) og passord
const char *ssid = "kristianIPHONE";
const char *password = "passord";

// Add your MQTT Broker IP address (mulig?: epstin.com)
// const char *mqtt_server = "IP_ADRESSE"; får ikke ip til å fungere...
const char *mqtt_server = "epstin.com";
const int mqtt_port = 1883;

// deklarerer navn og variabler
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

// NB: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc.html#adc-limitations
//  variabler led
const int blueLed = 5;
const int blueChannel = 5;
// led for wifi og mqtt kobling
const int mqttLed = 32;
const int mqttChannel = 2;

#define POT_METER 34

void setup_wifi()
{
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// funksjon som tar inn topic, melding og lengde
void callback(char *topic, byte *message, unsigned int length)
{
  // printer topic og melding i serial
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String dataMessage;

  // lager en variabel "dataMessage" og legger char sammen
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)message[i]);
    dataMessage += (char)message[i];
  }
  Serial.println();

  // Hvis vi motar en melding på esp32/output så ser vi etter "on" og "off" og gjør:
  if (String(topic) == "esp32/output")
  {
    Serial.print("Changing output to ");
    if (dataMessage == "on")
    {
      Serial.println("on");
      ledcWrite(blueChannel, 255);
    }
    else if (dataMessage == "off")
    {
      Serial.println("off");
      ledcWrite(blueChannel, 0);
    }
  }
}

// funksjon som setter opp led og channel (tar da inn ledPinne og ønsket kanal)
void setupLED(int ledPin, int channelLED)
{
  pinMode(ledPin, OUTPUT);
  ledcSetup(channelLED, 2000, 8);
  ledcAttachPin(ledPin, channelLED);
  ledcWrite(channelLED, 0);
}

void setup()
{
  Serial.begin(9600);
  // starter wifi:
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  setupLED(blueLed, blueChannel);
  setupLED(mqttLed, mqttChannel);
}

// funksjon som kjører til mqtt er koblet opp
void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client"))
    {
      Serial.println("connected");
      ledcWrite(mqttChannel, 255); // indikerer mqtt og wifi tilkobling
      // Subscribe
      client.subscribe("esp32/output");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void loop()
{
  // hvis clienten ikke er tilkoblet, kjør "reconnect"
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  // leser av potensiometeret og gjør om til verdi mellom 0 og 100 (brukes som testdata)
  int potMeterReading = analogRead(POT_METER);
  int battery = map(potMeterReading, 0, 4096, 0, 100);
  // ledcWrite(wifiChannel, battery);
  //   Serial.println(battery);

  long now = millis();
  // leser av hvert sekund
  if (now - lastMsg > 1000)
  {
    lastMsg = now;

    // fra "int" -> "string" -> "char*":
    String batteryString = String(battery);
    int str_len = batteryString.length() + 1;
    char char_array[str_len];
    // kopierer string over til char
    batteryString.toCharArray(char_array, str_len);

    Serial.print("Batteriprosent: ");
    Serial.print(battery);
    Serial.println("%");
    client.publish("esp32/battery", char_array);
  }
}
