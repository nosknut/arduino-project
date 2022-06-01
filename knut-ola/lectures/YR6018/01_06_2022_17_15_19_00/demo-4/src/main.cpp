// https://randomnerdtutorials.com/esp32-mqtt-publish-subscribe-arduino-ide/
#include <WiFi.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>

class Timer
{
private:
    unsigned long startTime;

public:
    Timer()
    {
        restart();
    }

    void restart()
    {
        startTime = millis();
    }

    unsigned long getElapsedTime()
    {
        return millis() - startTime;
    }

    bool isFinished(unsigned long duration)
    {
        return getElapsedTime() >= duration;
    }

    bool loopWait(unsigned long duration)
    {
        if (isFinished(duration))
        {
            restart();
            return true;
        }
        else
        {
            return false;
        }
    }
};

class SoftwareBattery
{
private:
    int maxCapacity = 100;
    int capacity = maxCapacity;

public:
    void reset()
    {
        capacity = maxCapacity;
    }

    int getCapacity()
    {
        if (capacity > 0)
        {
            capacity -= 1;
        }
        else
        {
            capacity = maxCapacity;
        }

        return capacity;
    }
};

String ssid = "Telenor1959bak";
String password = "yzefbjbqzgxqu";

// https://mntolia.com/10-free-public-private-mqtt-brokers-for-testing-prototyping/
const String mqtt_server = "test.mosquitto.org";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

String LED_TOPIC = "esp32/led";
String BATTERY_TOPIC = "esp32/battery";
String BATTERY_CAPACITY_TOPIC = "esp32/battery/capacity";

String topics[] = {
    LED_TOPIC,
    BATTERY_TOPIC,
    BATTERY_CAPACITY_TOPIC,
};

Timer timer;
SoftwareBattery battery;

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
        Serial.println("Subscribed to " + topic);
    }
}

void ensureMqttClientIsConnected()
{
    if (!client.connected())
    {
        // Loop until we're reconnected
        while (!client.connected())
        {
            Serial.print("Attempting MQTT connection...");
            // Attempt to connect
            if (client.connect(""))
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
}

void onLedMessage(String message)
{
    if (message == "on")
    {
        Serial.println("Turning led on");
    }

    if (message == "off")
    {
        Serial.println("Turning led off");
    }
}

void onBatteryMessage(String message)
{
    if (message == "reset")
    {
        Serial.println("Resetting battery");
        battery.reset();
    }
}

String stringFromByteArray(byte *message, unsigned int length)
{
    String result;
    for (int i = 0; i < length; i++)
    {
        result += (char)message[i];
    }
    return result;
}

void callback(char *topic, byte *message, unsigned int length)
{
    String topicString = String(topic);
    String messageString = stringFromByteArray(message, length);

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
    ensureMqttClientIsConnected();
    client.loop();

    if (timer.loopWait(3000))
    {
        int batteryCapacity = battery.getCapacity();
        String batteryCapacityString = String(batteryCapacity);

        client.publish(BATTERY_CAPACITY_TOPIC.c_str(), batteryCapacityString.c_str());
    }

    if (Serial.available())
    {
        String message = Serial.readString();
        client.publish("esp32/input", message.c_str());
    }
}
