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
typedef SerialNodeHandle Serial_NodeHandle;
typedef ros::NodeHandle_<Esp32WiFiHardware> WiFi_NodeHandle;

class MainRosserialBridge
{
private:
    bool wasConnected = false;

    Serial_NodeHandle inputNh;
    WiFi_NodeHandle outputNh;

    RosserialBridge<sensor_msgs::Imu> imuBridge = RosserialBridge<sensor_msgs::Imu>("imu");
    RosserialBridge<std_msgs::Int16> leftEncoderBridge = RosserialBridge<std_msgs::Int16>("left_ticks");
    RosserialBridge<std_msgs::Int16> rightEncoderBridge = RosserialBridge<std_msgs::Int16>("right_ticks");
    RosserialBridge<sensor_msgs::Range> irBridge = RosserialBridge<sensor_msgs::Range>("range_data");

public:
    MainRosserialBridge()
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
