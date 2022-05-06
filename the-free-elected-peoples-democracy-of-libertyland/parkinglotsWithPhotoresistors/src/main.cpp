// includes necessary libraries
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
// variables that represent last loops values
int previousParkingOne = 1;
int previousParkingTwo = 1;
int previousParkingThree = 1;
// variables used for timing
unsigned long previousTimeOne;
unsigned long previousTimeTwo;
unsigned long previousTimeThree;
// counter that represents seconds
int timeParkedAtOne;
int timeParkedAtTwo;
int timeParkedAtThree;

// pins that are used
#define ParkingOne 5
#define ParkingTwo 18
#define ParkingThree 19

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

// function that sends data to the parking_1 topic
void sendDataToTopicOne(int parkedInSlotOneState)
{
  int parkedInSlotOne = parkedInSlotOneState;                                                                                      // sends state 1 or 0
  String jsonOut = "{\"owner\": \"esp\", \"amount\": " + String(parkedInSlotOne) + ", \"time\": " + String(timeParkedAtOne) + "}"; // puts value in json string
  client.publish("esp32/parking_1", jsonOut.c_str());                                                                              // publishes to the topic
}

// function that sends data to the parking_2 topic with the same structur as sendDataToTopicOne
void sendDataToTopicTwo(int parkedInSlotTwoState)
{
  int parkedInSlotTwo = parkedInSlotTwoState;
  String jsonOut = "{\"owner\": \"esp\", \"amount\": " + String(parkedInSlotTwo) + ", \"time\": " + String(timeParkedAtTwo) + "}";
  client.publish("esp32/parking_2", jsonOut.c_str());
}

// function that sends data to the parking_3 topic with the same structur as sendDataToTopicOne
void sendDataToTopicThree(int parkedInSlotThreeState)
{
  int parkedInSlotThree = parkedInSlotThreeState;
  String jsonOut = "{\"owner\": \"esp\", \"amount\":" + String(parkedInSlotThree) + ", \"time\": " + String(timeParkedAtThree) + "}";
  client.publish("esp32/parking_3", jsonOut.c_str());
}

// timed interval for counting seconds parked at parking1
void paymentParkedAtOne()
{
  unsigned long now = millis();
  if (now - previousTimeOne >= 1000)
  {
    timeParkedAtOne++; // increases by one every second
    previousTimeOne = millis();
  }
}

// timed interval for counting seconds parked at parking2
void paymentParkedAtTwo()
{
  unsigned long now = millis();
  if (now - previousTimeTwo >= 1000)
  {
    timeParkedAtTwo++;
    previousTimeTwo = millis();
  }
}

// timed interval for counting seconds parked at parking3
void paymentParkedAtThree()
{
  unsigned long now = millis();
  if (now - previousTimeThree >= 1000)
  {
    timeParkedAtThree++;
    previousTimeThree = millis();
  }
}

// function that checks state on every parking
void parkingWithPhotoRes()
{
  /*reads if pin is high or low, if somthing is parked the pin is low
  and if it is available the pin is high*/
  int parkingStateOne = digitalRead(ParkingOne);
  int parkingStateTwo = digitalRead(ParkingTwo);
  int parkingStateThree = digitalRead(ParkingThree);

  Serial.println(parkingStateOne);
  Serial.println(parkingStateTwo);
  Serial.println(parkingStateThree);

  if (parkingStateOne == 0) // checks if something is parked on parking1
  {
    paymentParkedAtOne(); // counts seconds
  }

  if (parkingStateTwo == 0) // parking2
  {
    paymentParkedAtTwo();
  }

  if (parkingStateThree == 0) // parking3
  {
    paymentParkedAtThree();
  }

  if (parkingStateOne != previousParkingOne) // if a something moves on or of parking1
  {
    sendDataToTopicOne(parkingStateOne); // sends time parked and if available or taken
    Serial.println("Sent from one");
    timeParkedAtOne = 0; // sets time to zero for next parkingstime
  }

  if (parkingStateTwo != previousParkingTwo) // parking2
  {
    sendDataToTopicTwo(parkingStateTwo);
    Serial.println("Sent from two");
    timeParkedAtTwo = 0;
  }

  if (parkingStateThree != previousParkingThree) // parking3
  {
    sendDataToTopicThree(parkingStateThree);
    Serial.println("Sent from three");
    timeParkedAtThree = 0;
  }
  // registrers the loops pin readings
  previousParkingOne = parkingStateOne;
  previousParkingTwo = parkingStateTwo;
  previousParkingThree = parkingStateThree;
  delay(1000);
}

void loop()
{
  if (!client.connected()) // if not connected to MQTT broker
  {
    reconnect();
  }
  client.loop();         // keeps client running
  parkingWithPhotoRes(); // runs main
}
