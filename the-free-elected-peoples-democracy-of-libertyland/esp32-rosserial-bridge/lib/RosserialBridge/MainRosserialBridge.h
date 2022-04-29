#ifndef MainRosserialBridge_h
#define MainRosserialBridge_h
#include <Arduino.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <RosserialBridge.h>
#include <Esp32WiFiHardware.h>
#include <Esp32SerialHardware.h>
#include <SerialBridge.h>

// typedef ros::NodeHandle_<Esp32SerialHardware> Serial_NodeHandle;
typedef ros::NodeHandle_<Esp32WiFiHardware> WiFi_NodeHandle;

class ImuBridge : public RosserialBridge<sensor_msgs::Imu>
{
public:
    ImuBridge(String nodeName) : RosserialBridge<sensor_msgs::Imu>(nodeName) {}

    void mapMessages(sensor_msgs::Imu &inMsg, sensor_msgs::Imu &outMsg)
    {
        outMsg.header.stamp = inMsg.header.stamp;
        outMsg.orientation.x = inMsg.orientation.x;
        outMsg.orientation.y = inMsg.orientation.y;
        outMsg.orientation.z = inMsg.orientation.z;
        outMsg.orientation.w = inMsg.orientation.w;
        outMsg.angular_velocity.x = inMsg.angular_velocity.x;
        outMsg.angular_velocity.y = inMsg.angular_velocity.y;
        outMsg.angular_velocity.z = inMsg.angular_velocity.z;
        outMsg.linear_acceleration.x = inMsg.linear_acceleration.x;
        outMsg.linear_acceleration.y = inMsg.linear_acceleration.y;
        outMsg.linear_acceleration.z = inMsg.linear_acceleration.z;
    }
};

class RangeBridge : public RosserialBridge<sensor_msgs::Range>
{
public:
    RangeBridge(String nodeName) : RosserialBridge<sensor_msgs::Range>(nodeName) {}

    void mapMessages(sensor_msgs::Range &inMsg, sensor_msgs::Range &outMsg)
    {
        outMsg.header.stamp = inMsg.header.stamp;
        outMsg.radiation_type = inMsg.radiation_type;
        outMsg.field_of_view = inMsg.field_of_view;
        outMsg.min_range = inMsg.min_range;
        outMsg.max_range = inMsg.max_range;
        outMsg.range = inMsg.range;
    }
};

class Int16Bridge : public RosserialBridge<std_msgs::Int16>
{
public:
    Int16Bridge(String nodeName) : RosserialBridge<std_msgs::Int16>(nodeName) {}

    void mapMessages(std_msgs::Int16 &inMsg, std_msgs::Int16 &outMsg)
    {
        outMsg.data = inMsg.data;
        Serial.println(String(outMsg.data) + "   " + String(inMsg.data));
    }
};

class MainRosserialBridge
{
private:
    bool wasConnected = false;

    SerialConnection inputNh;
    WiFi_NodeHandle outputNh;

    ImuBridge imuBridge = ImuBridge("imu");
    Int16Bridge leftEncoderBridge = Int16Bridge("left_ticks");
    Int16Bridge rightEncoderBridge = Int16Bridge("right_ticks");
    RangeBridge irBridge = RangeBridge("range_data");

public:
    MainRosserialBridge() : inputNh(&Serial)
    {
    }

    void setup(IPAddress serverIpAddress, long baudRate, uint16_t serverPort = 11411)
    {
        // Set the connection to rosserial socket server
        outputNh.getHardware()->setConnection(serverIpAddress, serverPort);
        outputNh.initNode();

        inputNh.getHardware()->setBaud(baudRate);
        inputNh.initNode();

        imuBridge.setup(inputNh, outputNh);
        leftEncoderBridge.setup(inputNh, outputNh);
        rightEncoderBridge.setup(inputNh, outputNh);
        irBridge.setup(inputNh, outputNh);
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
                wasConnected = true;
            }

            imuBridge.loop(inputNh, outputNh);
            leftEncoderBridge.loop(inputNh, outputNh);
            rightEncoderBridge.loop(inputNh, outputNh);
            irBridge.loop(inputNh, outputNh);
        }
        else
        {
            if (wasConnected)
            {
                Serial.println("Disconnected");
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
        inputNh.spinOnce();
        outputNh.spinOnce();
    }
};

#endif
