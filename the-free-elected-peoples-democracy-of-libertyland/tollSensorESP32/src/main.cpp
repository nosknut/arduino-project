/*
This code uses an ESP32 and HCsr04 sensor to register passings
*/

// includes necessary libraries
#include <HCSR04.h>
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

// Wifi name (ssid) and password
const char *ssid = "kristianIPHONE";
const char *password = "";

// Adds MQTT Broker IP adress
// const char *mqtt_server = "IP_ADRESS";
const char *mqtt_server = "172.20.10.10";
const int mqtt_port = 1883;

// declair names and variables
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

// flag for registering passing
int paymentflag = 0;
// consts to LED
const int LED = 32;
const int channelLED = 0;

/*function that sets up and takes input from HCsr04 sensoren
 class HCSR04 (trig pin , echo pin)*/
HCSR04 hc(5, 18);

// function that connects to wifi
void setup_wifi()
{
  delay(10);
  // Start by connecting to wifi
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) // while trying to connect
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
  ledcWrite(channelLED, 0); // Start with led off
  // start wifi:
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
}
// function that makes sure that the ESP32 stays connected to wifi
void reconnect()
{
  // Loop until connected to wifi
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // try to connect
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
      // wait 5 seconds to not spam message queue
      delay(5000);
    }
  }
}

// function that checks hcsr04 and sends data
void tollSensor()
{
  float Distance = hc.dist(); // hc.dist returns distance in cm
  Serial.println(Distance);
  if (Distance <= 10) // if an object is not infront of sensor
  {
    paymentflag = 1; // flag is set high to be able to go into if
    ledcWrite(channelLED, 0);
  }
  if (Distance > 10) // if an object is in front of sensor
  {
    if (paymentflag == 1) // makes sure that signal is only sent once in the passing of the toll
    {
      int moneyPaidToToll = -50;
      String jsonOut = "{\"owner\": \"Zumo\", \"amount\": " + String(moneyPaidToToll) + "}"; // turns value to string
      client.publish("esp32/toll", jsonOut.c_str());                                         // sends json string to the topic esp32/toll
      ledcWrite(channelLED, 255);                                                            // indicates that the string has been published
      delay(2000);
      paymentflag = 0; // sets  the flag t 0 so that standing still in front of sensor is not registered
    }
    else // if object stands still in front of sensor
    {
      ledcWrite(channelLED, 0);
    }
  }
  delay(60); // delay for stable hcsr04 readings
}

void loop()
{
  if (!client.connected()) // if not connected to MQTT broker
  {
    reconnect();
  }
  client.loop(); // keeps client running
  tollSensor();  // runs main
}