// inkluderer nødvendige biblioteker
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

// Wifi navn (ssid) og passord
const char *ssid = "zenbook-kristian";
const char *password = "hansen123";

// Legger til MQTT Broker IP addresse
// const char *mqtt_server = "IP_ADRESSE";
const char *mqtt_server = "10.22.223.162";
const int mqtt_port = 1883;

// deklarerer navn og variabler
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;
// verdier som representer forrige loops verdier
int previousParkingOne = 1;
int previousParkingTwo = 1;
int previousParkingThree = 1;
// verdier brukt til timing
unsigned long previousTimeOne;
unsigned long previousTimeTwo;
unsigned long previousTimeThree;
// counter som representerer sekunder
int timeParkedAtOne;
int timeParkedAtTwo;
int timeParkedAtThree;

// portene som blir brukt
#define ParkingOne 5
#define ParkingTwo 18
#define ParkingThree 19

// funksjon som kobler til wifi
void setup_wifi()
{
  delay(10);
  // Starter med å koble til wifi
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) // mens den prøver å koble til
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
  // starter wifi:
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
}

// funksjon som sørger for at esp32en forblir koblet til wifiet
void reconnect()
{
  // Loop frem til en koblet på igjen
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // prøve å koble til
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
      // Vent 5 sekunder for å ikke bombadere message queuen
      delay(5000);
    }
  }
}

// funksjon som sender data til parking_1 topicet
void sendDataToTopicOne(int parkedInSlotOneState)
{
  int parkedInSlotOne = parkedInSlotOneState;                                                                                      // sender staten 1 eller 0
  String jsonOut = "{\"owner\": \"esp\", \"amount\": " + String(parkedInSlotOne) + ", \"time\": " + String(timeParkedAtOne) + "}"; // setter i json string
  client.publish("esp32/parking_1", jsonOut.c_str());                                                                              // publisher til topicet
}

// funksjon som sender data til parking_2 topicet med samme struktur som sendDataToTopicOne
void sendDataToTopicTwo(int parkedInSlotTwoState)
{
  int parkedInSlotTwo = parkedInSlotTwoState;
  String jsonOut = "{\"owner\": \"esp\", \"amount\": " + String(parkedInSlotTwo) + ", \"time\": " + String(timeParkedAtTwo) + "}";
  client.publish("esp32/parking_2", jsonOut.c_str());
}

// funksjon som sender data til parking_3 topicet med samme struktur som sendDataToTopicOne
void sendDataToTopicThree(int parkedInSlotThreeState)
{
  int parkedInSlotThree = parkedInSlotThreeState;
  String jsonOut = "{\"owner\": \"esp\", \"amount\":" + String(parkedInSlotThree) + ", \"time\": " + String(timeParkedAtThree) + "}";
  client.publish("esp32/parking_3", jsonOut.c_str());
}

// timed interval for å telle sekunder parkert på 1
void paymentParkedAtOne()
{
  unsigned long now = millis();
  if (now - previousTimeOne >= 1000)
  {
    timeParkedAtOne++; //øker med 1 hvert sekund
    previousTimeOne = millis();
  }
}

// timed interval for å telle sekunder parkert på 2
void paymentParkedAtTwo()
{
  unsigned long now = millis();
  if (now - previousTimeTwo >= 1000)
  {
    timeParkedAtTwo++;
    previousTimeTwo = millis();
  }
}

// timed interval for å telle sekunder parkert på 3
void paymentParkedAtThree()
{
  unsigned long now = millis();
  if (now - previousTimeThree >= 1000)
  {
    timeParkedAtThree++;
    previousTimeThree = millis();
  }
}

// funksjon som sjekker status på alle parkeringsplassene
void parkingWithPhotoRes()
{
  /*leser om pinnen er høy eller lav, hvis det er noen parkert er pinnen lav
  og hvis den er ledig står den high*/
  int parkingStateOne = digitalRead(ParkingOne);
  int parkingStateTwo = digitalRead(ParkingTwo);
  int parkingStateThree = digitalRead(ParkingThree);

  Serial.println(parkingStateOne);
  Serial.println(parkingStateTwo);
  Serial.println(parkingStateThree);

  if (parkingStateOne == 0) // hvis noen står på parkeringsplass 1
  {
    paymentParkedAtOne(); // teller sekunder
  }

  if (parkingStateTwo == 0) // parkeringsplass2
  {
    paymentParkedAtTwo();
  }

  if (parkingStateThree == 0) // parkeringsplass3
  {
    paymentParkedAtThree();
  }

  if (parkingStateOne != previousParkingOne) // hvis en bil kjører på eller av parkering1
  {
    sendDataToTopicOne(parkingStateOne); // sender tid stått og om ledig eller opptatt
    Serial.println("Sent from one");
    timeParkedAtOne = 0; // setter tiden til null for neste parkeringstid
  }

  if (parkingStateTwo != previousParkingTwo) // parkering2
  {
    sendDataToTopicTwo(parkingStateTwo);
    Serial.println("Sent from two");
    timeParkedAtTwo = 0;
  }

  if (parkingStateThree != previousParkingThree) // parkering3
  {
    sendDataToTopicThree(parkingStateThree);
    Serial.println("Sent from three");
    timeParkedAtThree = 0;
  }
  // registrerer loopens parkering verdier
  previousParkingOne = parkingStateOne;
  previousParkingTwo = parkingStateTwo;
  previousParkingThree = parkingStateThree;
  delay(1000);
}

void loop()
{
  if (!client.connected()) // hvis den ikke er koblet til
  {
    reconnect();
  }
  client.loop(); // holde klienten gående
  parkingWithPhotoRes();
}
