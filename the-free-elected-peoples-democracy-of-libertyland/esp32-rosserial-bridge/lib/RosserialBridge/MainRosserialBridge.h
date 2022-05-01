#ifndef MainRosserialBridge_h
#define MainRosserialBridge_h
#include <Arduino.h>
#include <ros/node_handle.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <RosserialSubscriberBridge.h>
#include <RosserialPublisherBridge.h>
#include <Esp32WiFiHardware.h>
#include <ArduinoJson.h>
#include <SerialClass.h>

// typedef ros::NodeHandle_<Esp32SerialHardware> Serial_NodeHandle;
typedef ros::NodeHandle_<Esp32WiFiHardware> WiFi_NodeHandle;

class ImuBridge : public RosserialSubscriberBridge<sensor_msgs::Imu, WiFi_NodeHandle>
{
public:
    ImuBridge(String nodeName, String frameId) : RosserialSubscriberBridge<sensor_msgs::Imu, WiFi_NodeHandle>(nodeName)
    {
        this->msg.header.frame_id = frameId.c_str();
    }

    void mapMessages(WiFi_NodeHandle &nh, JsonDocument &inDoc, sensor_msgs::Imu &outMsg)
    {
        // TODO: Add covariances
        outMsg.header.stamp = nh.now();
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

class RangeBridge : public RosserialSubscriberBridge<sensor_msgs::Range, WiFi_NodeHandle>
{
public:
    RangeBridge(String nodeName, String frameId) : RosserialSubscriberBridge<sensor_msgs::Range, WiFi_NodeHandle>(nodeName)
    {
        this->msg.header.frame_id = frameId.c_str();
    }

    void mapMessages(WiFi_NodeHandle &nh, JsonDocument &inDoc, sensor_msgs::Range &outMsg)
    {
        outMsg.header.stamp = nh.now();
        outMsg.radiation_type = sensor_msgs::Range::INFRARED;
        outMsg.field_of_view = 0.001;
        outMsg.min_range = 0.03;
        outMsg.max_range = 0.4;
        outMsg.range = inDoc["r"];
    }
};

class Int16Bridge : public RosserialSubscriberBridge<std_msgs::Int16, WiFi_NodeHandle>
{
public:
    Int16Bridge(String nodeName) : RosserialSubscriberBridge<std_msgs::Int16, WiFi_NodeHandle>(nodeName) {}

    void mapMessages(WiFi_NodeHandle &nh, JsonDocument &inDoc, std_msgs::Int16 &outMsg)
    {
        outMsg.data = inDoc["data"];
    }
};

class Int16SubscriberBridge : public RosserialSubscriberBridge<std_msgs::Int16, WiFi_NodeHandle>
{
public:
    Int16SubscriberBridge(String nodeName) : RosserialSubscriberBridge<std_msgs::Int16, WiFi_NodeHandle>(nodeName) {}

    void mapMessages(WiFi_NodeHandle &nh, JsonDocument &inDoc, std_msgs::Int16 &outMsg)
    {
        outMsg.data = inDoc["data"];
    }
};

class Float32PublisherBridge : public RosserialPublisherBridge<std_msgs::Float32, WiFi_NodeHandle>
{
public:
    Float32PublisherBridge(String nodeName) : RosserialPublisherBridge<std_msgs::Float32, WiFi_NodeHandle>(nodeName)
    {
        this->outputDocument["topic"] = nodeName;
        this->outputDocument["data"] = (int16_t)0;
    }

    void mapMessages(const std_msgs::Float32 &inMsg, JsonDocument &outDoc)
    {
        outDoc["data"] = (int16_t)inMsg.data;
    }
};

class MainRosserialBridge
{
private:
    bool wasConnected = false;

    DynamicJsonDocument inputDoc = DynamicJsonDocument(255);
    SerialClass inputStream = DATA_SERIAL_CLASS;
    WiFi_NodeHandle &outputNh;

    ImuBridge imuBridge = ImuBridge("imu", "imu_link");

    Int16SubscriberBridge leftEncoderBridge = Int16SubscriberBridge("left_ticks");
    Int16SubscriberBridge rightEncoderBridge = Int16SubscriberBridge("right_ticks");

    RangeBridge flRangeBridge = RangeBridge("fl_range", "fl_range_link");
    RangeBridge frRangeBridge = RangeBridge("fr_range", "fr_range_link");
    RangeBridge lRangeBridge = RangeBridge("l_range", "l_range_link");
    RangeBridge rRangeBridge = RangeBridge("r_range", "r_range_link");

    Float32PublisherBridge leftMotorBridge = Float32PublisherBridge("lmotor_cmd");
    Float32PublisherBridge rightMotorBridge = Float32PublisherBridge("rmotor_cmd");

public:
    MainRosserialBridge(WiFi_NodeHandle &outputNh) : outputNh(outputNh)
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

        flRangeBridge.setup(outputNh);
        frRangeBridge.setup(outputNh);
        lRangeBridge.setup(outputNh);
        rRangeBridge.setup(outputNh);

        leftMotorBridge.setup(outputNh);
        rightMotorBridge.setup(outputNh);
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
                Serial.println("Connected!");
                outputNh.loginfo("Connected to rosserial!");
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

                    flRangeBridge.loop(inputDoc, outputNh);
                    frRangeBridge.loop(inputDoc, outputNh);
                    lRangeBridge.loop(inputDoc, outputNh);
                    rRangeBridge.loop(inputDoc, outputNh);
                }
                else
                {
                    String errorMessage = String("Deserialization error: " + String(err.c_str()));
                    // NB! Passing a string directly to this logger will cause compilation errors
                    outputNh.logdebug(errorMessage.c_str());
                }
            }
        }
        else
        {
            if (wasConnected)
            {
                Serial.println("Disconnected");
                outputNh.logerror("Disconnected from rosserial");
                wasConnected = false;
            }
            else
            {
                Serial.println("Not Connected");
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
