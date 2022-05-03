// including all the necessary libraries
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <Arduino_JSON.h>
#include "MyHeader.h"  // Including header file

// Defining constants
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);  // OLED setup

//MQTT setup
WiFiClient espClient;
PubSubClient client(espClient);


void setup() {
  Serial.begin(115200); //Starting serial

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }
  // Clear the buffer
  display.clearDisplay();



  WiFi.begin(ssid, password); // Connect to WiFI
  Serial.println("Connecting");
  // Display dots in serial while not connected
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Printing network status
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.println("It will take 5 seconds before publishing the first reading.");

  // MQTT setup
  client.setServer(mqtt_server, mqtt_port);
}

void loop() {
  // Connect MQTT
  if (!client.connected()) {
    client.connect("ESP32Client");
  }
  client.loop();


  // Runs at a set interval
  if ((millis() - lastTime) > timerDelay) {
    if (WiFi.status() == WL_CONNECTED) {
      // API path
      

      // Reading the JSON file
      jsonBuffer = httpGETRequest(serverPath.c_str());
      Serial.println(jsonBuffer);
      JSONVar myObject = JSON.parse(jsonBuffer);

      // JSON.typeof(jsonVar) can be used to get the type of the var
      if (JSON.typeof(myObject) == "undefined") {
        Serial.println("Parsing input failed!");
        return;
      }

      // Printing some data from JSON object
      Serial.print("JSON object = ");
      Serial.println(myObject);
      Serial.print("Temperature: ");
      Serial.println(myObject["main"]["temp"]);
      Serial.print("Clouds: ");
      Serial.println(myObject["clouds"]["all"]);
      Serial.print("Wind Speed: ");
      Serial.println(myObject["wind"]["speed"]);

      // Calculates wind- and solar power to a precentage
      int solarPower =  100 - int(myObject["clouds"]["all"]);
      int windPower = map(long(myObject["wind"]["speed"]), 0, 15, 0, 100);

      // Reads temp sensor and calculates thermal procentage
      float thermal = (((analogRead(thermalPin)) / 1023.0) - 0.5) * 100;
      Serial.print("thermal = ");
      Serial.println(thermal);
      int thermalPower = map(long(thermal), 0, 40, 0, 100);

      // Calculates energy pricing based on power sources
      int energyPrice = 100 - int((solarPower + windPower + thermalPower / 2) / 3);

      // Clear the buffer
      display.clearDisplay();

      display.setTextSize(1);             // Normal 1:1 pixel scale
      display.setTextColor(SSD1306_WHITE);        // Draw white text
      display.setCursor(0, 32);            // Start at top-left corner

      // Displaying data on OLED
      display.print("Solar Power:");
      display.print(solarPower);
      display.println("%");

      display.print("Wind Power:");
      display.print(windPower);
      display.println("%");

      display.print("Thermal Power:");
      display.print(thermalPower);
      display.println("%");

      display.print("Price per kWh:");
      display.print(energyPrice);
      display.println("$");

      // Displaying weather symbol based on cloudyness
      if (int(myObject["clouds"]["all"]) < 25) {
        display.drawBitmap(96, 0, sunny, 32, 32, 1);
      }
      else if (int(myObject["clouds"]["all"]) < 75) {
        display.drawBitmap(89, 0, sunnyCloud, 39, 32, 1);
      }
      else {
        display.drawBitmap(86, 0, cloudy, 42, 32, 1);
      }
      
      display.display(); // Display on OLED

      // Sending weather data with MQTT, by converting the integers to arrays.
      String energyString = String(energyPrice);
      int str_len = energyString.length() + 1;
      char energy_array[str_len];
      // kopierer string over til char
      energyString.toCharArray(energy_array, str_len);
      client.publish("esp32/weather/energyprice", energy_array); // Publishes the array to a given topic

      String windString = String(windPower);
      char wind_array[str_len];
      // kopierer string over til char
      windString.toCharArray(wind_array, str_len);
      client.publish("esp32/weather/windpower", wind_array);

      String solarString = String(solarPower);
      char solar_array[str_len];
      // kopierer string over til char
      solarString.toCharArray(solar_array, str_len);
      client.publish("esp32/weather/solarpower", solar_array);

      String thermalString = String(thermalPower);
      char thermal_array[str_len];
      // kopierer string over til char
      thermalString.toCharArray(thermal_array, str_len);
      Serial.println(thermal_array);
      client.publish("esp32/weather/thermalpower", thermal_array);

    }
    // If WiFi disconnects
    else {
      Serial.println("WiFi Disconnected");
    }
    lastTime = millis();  // Updates timer
  }
}

// Function that sends a http request to a website, and fetches its content
String httpGETRequest(const char* serverName) {
  WiFiClient client;
  HTTPClient http;

  // Your Domain name with URL path or IP address with path
  http.begin(client, serverName);

  // Send HTTP POST request
  int httpResponseCode = http.GET();

  String payload = "{}";

  if (httpResponseCode > 0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = http.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  http.end();

  return payload;
}
