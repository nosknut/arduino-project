#include <HCSR04.h>
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

// Wifi navn (ssid) og passord
const char *ssid = "esp";
const char *password = "12345678";

// Add your MQTT Broker IP address (mulig?: epstin.com)
// const char *mqtt_server = "IP_ADRESSE"; får ikke ip til å fungere...
const char *mqtt_server = "10.24.3.237";
const int mqtt_port = 1883;

// deklarerer navn og variabler
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

// flag for registering passing
int paymentflag = 0;
// consts for LED
const int LED = 32;
const int channelLED = 0;

HCSR04 hc(5, 18); // initialisation class HCSR04 (trig pin , echo pin)

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

void setup()
{
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  ledcSetup(channelLED, 2000, 8);
  ledcAttachPin(LED, channelLED);
  ledcWrite(channelLED, 0); // Turns the light off at start
  // starter wifi:
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
}

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

void tollSensor()
{
  float Distance = hc.dist(); // return current distance (cm) in serial
  Serial.println(Distance);
  if (Distance <= 10)
  {
    paymentflag = 1;
    ledcWrite(channelLED, 0);
  }
  if (Distance > 10)
  {
    if (paymentflag == 1)
    {
      const int moneyPaidToToll = 50;
      String moneyString = String(moneyPaidToToll);
      int str_len = moneyString.length() + 1;
      char char_array[str_len];
      // kopierer string over til char
      moneyString.toCharArray(char_array, str_len);
      client.publish("esp32/battery", char_array);
      ledcWrite(channelLED, 255);
      delay(2000);
      paymentflag = 0;
    }
    else
    {
      ledcWrite(channelLED, 0);
    }
  }
  delay(60);
}

void loop()
{
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();
  tollSensor();
}