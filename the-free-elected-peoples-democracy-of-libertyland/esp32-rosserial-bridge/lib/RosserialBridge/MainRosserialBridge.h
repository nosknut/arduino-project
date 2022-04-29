#ifndef MainRosserialBridge_h
#define MainRosserialBridge_h
#include <Arduino.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <RosserialBridge.h>
#include <Esp32WiFiHardware.h>
#include <ArduinoJson.h>
#include <SerialClass.h>

// typedef ros::NodeHandle_<Esp32SerialHardware> Serial_NodeHandle;
typedef ros::NodeHandle_<Esp32WiFiHardware> WiFi_NodeHandle;

class ImuBridge : public RosserialBridge<sensor_msgs::Imu>
{
public:
    ImuBridge(String nodeName) : RosserialBridge<sensor_msgs::Imu>(nodeName) {}

    void mapMessages(JsonDocument &inDoc, sensor_msgs::Imu &outMsg)
    {
        // TODO: Add covariances
        outMsg.orientation.x = inDoc["o"]["x"];
        outMsg.orientation.y = inDoc["o"]["y"];
        outMsg.orientation.z = inDoc["o"]["z"];
        outMsg.orientation.w = inDoc["o"]["w"];
        outMsg.angular_velocity.x = inDoc["av"]["x"];
        outMsg.angular_velocity.y = inDoc["av"]["y"];
        outMsg.angular_velocity.z = inDoc["av"]["z"];
        outMsg.linear_acceleration.x = inDoc["la"]["x"];
        outMsg.linear_acceleration.y = inDoc["la"]["y"];
        outMsg.linear_acceleration.z = inDoc["la"]["z"];
    }
};

class RangeBridge : public RosserialBridge<sensor_msgs::Range>
{
public:
    RangeBridge(String nodeName) : RosserialBridge<sensor_msgs::Range>(nodeName) {}

    void mapMessages(JsonDocument &inDoc, sensor_msgs::Range &outMsg)
    {
        outMsg.radiation_type = sensor_msgs::Range::INFRARED;
        outMsg.field_of_view = 0.001;
        outMsg.min_range = 0.03;
        outMsg.max_range = 0.4;
        outMsg.range = inDoc["range"];
    }
};

class Int16Bridge : public RosserialBridge<std_msgs::Int16>
{
public:
    Int16Bridge(String nodeName) : RosserialBridge<std_msgs::Int16>(nodeName) {}

    void mapMessages(JsonDocument &inDoc, std_msgs::Int16 &outMsg)
    {
        outMsg.data = inDoc["data"];
    }
};

class MainRosserialBridge
{
private:
    bool wasConnected = false;

    DynamicJsonDocument inputDoc = DynamicJsonDocument(255);
    SerialClass inputStream = DATA_SERIAL_CLASS;
    WiFi_NodeHandle outputNh;

    ImuBridge imuBridge = ImuBridge("imu");
    Int16Bridge leftEncoderBridge = Int16Bridge("left_ticks");
    Int16Bridge rightEncoderBridge = Int16Bridge("right_ticks");
    RangeBridge irBridge = RangeBridge("range_data");

public:
    MainRosserialBridge()
    {
    }

    void setup(IPAddress serverIpAddress, long baudRate, uint16_t serverPort = 11411)
    {
        // Set the connection to rosserial socket server
        outputNh.getHardware()->setConnection(serverIpAddress, serverPort);
        outputNh.initNode();

        inputStream.begin(baudRate);

        imuBridge.setup(outputNh);
        leftEncoderBridge.setup(outputNh);
        rightEncoderBridge.setup(outputNh);
        irBridge.setup(outputNh);
    }

    void loop()
    {

        if (outputNh.connected())
        {
            // Run publishers
            // The publishers for this class
            // are bridges created in setup()
            if (!wasConnected)
            {
                // Serial.println("Connected!");
                wasConnected = true;
            }

            if (inputStream.available())
            {
                // Read the JSON document from the "link" serial port
                DeserializationError err = deserializeJson(inputDoc, inputStream);

                if (err == DeserializationError::Ok)
                {
                    imuBridge.loop(inputDoc, outputNh);
                    leftEncoderBridge.loop(inputDoc, outputNh);
                    rightEncoderBridge.loop(inputDoc, outputNh);
                    irBridge.loop(inputDoc, outputNh);
                }
                else
                {
                    // Print error to the "debug" serial port
                    Serial.print("deserializeJson() returned ");
                    Serial.println(err.c_str());

                    // Flush all bytes in the inputStream serial port buffer
                    while (inputStream.available() > 0)
                    {
                        inputStream.read();
                    }
                }
            }
        }
        else
        {
            if (wasConnected)
            {
                // Serial.println("Disconnected");
                wasConnected = false;
            }
            else
            {
                // Serial.println("Not Connected");
                delay(1000);
            }
        }
        // The RosPublisherBridge has no loop function
        // Regarding the line below:
        // nh.spinOnce() is the function that updates rosserial
        outputNh.spinOnce();
    }
};

#endif
