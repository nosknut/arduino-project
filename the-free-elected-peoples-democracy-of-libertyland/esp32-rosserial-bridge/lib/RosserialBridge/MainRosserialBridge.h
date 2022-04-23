#ifndef MainRosserialBridge_h
#define MainRosserialBridge_h
#include <Arduino.h>
#include <ros.h>
#include <ESP8266WiFi.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <RosserialBridge.h>

class MainRosserialBridge
{
private:
    // Set the rosserial socket server IP address
    IPAddress server = IPAddress(192, 168, 1, 1);

    AVR_ATmega328P_NodeHandle inputNh;
    ros::NodeHandle outputNh;

    RosserialBridge<sensor_msgs::Imu> imuBridge = RosserialBridge<sensor_msgs::Imu>("imu");
    RosserialBridge<std_msgs::Int16> leftEncoderBridge = RosserialBridge<std_msgs::Int16>("left_ticks");
    RosserialBridge<std_msgs::Int16> rightEncoderBridge = RosserialBridge<std_msgs::Int16>("right_ticks");
    RosserialBridge<sensor_msgs::Range> irBridge = RosserialBridge<sensor_msgs::Range>("range_data");

public:
    MainRosserialBridge()
    {
    }

    void setup(long baudRate, uint16_t serverPort = 11411)
    {
        // Set the connection to rosserial socket server
        outputNh.getHardware()->setConnection(server, serverPort);
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
        }
        else
        {
            Serial.println("Not Connected");
            delay(100);
        }
        // The RosPublisherBridge has no loop function
        // Regarding the line below:
        // nh.spinOnce() is the function that updates rosserial
        inputNh.spinOnce();
        outputNh.spinOnce();
    }
};

#endif
