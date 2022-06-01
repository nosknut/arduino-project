#include <WiFi.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>

String ssid = "Telenor1959bak";
String password = "yzefbjbqzgxqu";

// https://mntolia.com/10-free-public-private-mqtt-brokers-for-testing-prototyping/
const String mqtt_server = "mqtt.eclipse.org";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

String LED_TOPIC = "esp32/led";
String BATTERY_TOPIC = "esp32/battery";

String topics[] = {
    LED_TOPIC,
    BATTERY_TOPIC,
};

void ensureWifiIsConnected()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.print("WiFi connecting to ");
        Serial.println(ssid);

        WiFi.begin(ssid.c_str(), password.c_str());

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
}

void subscribeToTopics()
{
    for (String topic : topics)
    {
        client.subscribe(topic.c_str());
    }
}

void ensureMqttClientIsConnected()
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
            subscribeToTopics();
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

void onLedMessage(String message)
{
    if (message == "on")
    {
        Serial.println("Turning led on");
    }
    else if (message == "off")
    {
        Serial.println("Turning led off");
    }
}

void onBatteryMessage(String message)
{
    if (message == "reset")
    {
        Serial.println("Resetting battery");
    }
}

void callback(char *topic, byte *message, unsigned int length)
{
    String topicString = String(topic);
    String messageString = String((char *)message);

    if (topicString == LED_TOPIC)
    {
        onLedMessage(messageString);
    }

    if (topicString == BATTERY_TOPIC)
    {
        onBatteryMessage(messageString);
    }
}

void setup()
{
    Serial.begin(9600);
    ensureWifiIsConnected();
    client.setServer(mqtt_server.c_str(), mqtt_port);
    client.setCallback(callback);
    ensureMqttClientIsConnected();
}

void loop()
{
    ensureWifiIsConnected();

    if (Serial.available())
    {
        String message = Serial.readString();
        client.publish("esp32/input", message.c_str());
    }
}
