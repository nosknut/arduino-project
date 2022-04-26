// enkel versjon
// inkluderer Wifi og Webserver bibliotek
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_I2CDevice.h>

// ssiden og passordet til Wi-Fien ESP32en skal koble til
const char *ssid = "KristianIPHONE";
const char *password = "kristian";

// Konfigurer en server på ESP32en med portnummer 80
WebServer server(80);

#define POT_PIN 35   // pinnen til Potmeter
#define PHOTO_PIN 34 // pinnen til Photoresistor

// setter opp en string som skal inneholde html og sensordata
String refreshWebData()
{
  int sensorPOTMETER = analogRead(POT_PIN);
  int sensorPHOTORES = analogRead(PHOTO_PIN);

  // string med default HTML kode på nettsiden
  String HTMLdata = "<!DOCTYPE html>"
                    "<html>"
                    "<head>"
                    "<title>Sensordata</title>"
                    "<meta http-equiv='refresh' content='0.5'>"
                    "<html>"
                    "<body>"
                    "<h1>Sensordata:</h1>"
                    "</body>"
                    "</html>";

  // legger til sensordataen til i HTML koden
  HTMLdata += "<h2>POTMETER: ";
  HTMLdata += sensorPOTMETER;
  HTMLdata += "</h2>";

  HTMLdata += "<h2>PHOTORES: ";
  HTMLdata += sensorPHOTORES;
  HTMLdata += "</h2>";

  return HTMLdata; // returner HTML stringen
}

// funksjon for når en klinet er koblet til
void handle_root()
{
  // 200 status kode, "text/html" data type, hva som sendes
  server.send(200, "text/html", refreshWebData());
}

void setup()
{
  Serial.begin(9600); // iverksetter serial

  // Kobler til det valgte wifiet
  WiFi.begin(ssid, password);

  // Venter i while frem til forbindelsen til wifien er opprettet
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print(".");
  } // end while !=WL_connected

  // printer at tilkobling gikk bra og printer IP adressen
  Serial.println("");
  Serial.println("WiFi connected successfully");
  Serial.print("Got IP: ");
  Serial.println(WiFi.localIP());

  // når serveren er på skjer void handle root som sender data
  server.on("/", handle_root);

  // begynner serveren
  server.begin();
  Serial.println("HTTP server started");
  delay(100);
}

void loop()
{
  // holder nettsiden oppe med automatisk refresh av siden hver gang void loop kjører
  server.handleClient();
  delay(10);
}
