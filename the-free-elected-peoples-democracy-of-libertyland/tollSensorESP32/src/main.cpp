// inkluderer nødvendige biblioteker
#include <HCSR04.h>
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

// flag for å registrere passering
int paymentflag = 0;
// consts til LED
const int LED = 32;
const int channelLED = 0;

/*funksjon som setter opp og tar input fra HCsr04 sensoren
 class HCSR04 (trig pin , echo pin)*/
HCSR04 hc(5, 18);

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
  pinMode(LED, OUTPUT);
  ledcSetup(channelLED, 2000, 8);
  ledcAttachPin(LED, channelLED);
  ledcWrite(channelLED, 0); // Starter med lyset av
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
      client.subscribe("esp32/output");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Vent 5 sekunder for å ikke bombadere message queue
      delay(5000);
    }
  }
}

// funksjonen som sjekker hcsr04 og sender dataen
void tollSensor()
{
  float Distance = hc.dist(); // hc.dist returner avstand i cm
  Serial.println(Distance);
  if (Distance <= 10) // hvis et objekt ikke er foran sensoren
  {
    paymentflag = 1; // flagget settes høyt slik at den kan gå i if
    ledcWrite(channelLED, 0);
  }
  if (Distance > 10) // hvis et objekt er foran sensoren
  {
    if (paymentflag == 1) // sørger for at signal kun sendes en gang i passeringen av bommen
    {
      int moneyPaidToToll = -50;                                                             // sender en verdi -50
      String jsonOut = "{\"owner\": \"Zumo\", \"amount\": " + String(moneyPaidToToll) + "}"; // gjør om verdien til string
      client.publish("esp32/toll", jsonOut.c_str());                                         // sender json string til topicet esp32/toll
      ledcWrite(channelLED, 255);                                                            // indikerer at stringen har blitt published
      delay(2000);
      paymentflag = 0; // setter flagget til 0 for å kun registrere selve passeringen og ikke om den står i ro
    }
    else // hvis bilen blir stående foran sensoren
    {
      ledcWrite(channelLED, 0);
    }
  }
  delay(60); // delay for stabil hcsr04 lesninger
}

void loop()
{
  if (!client.connected()) // hvis den ikke er koblet til
  {
    reconnect();
  }
  client.loop(); // holde klienten gående
  tollSensor();
}