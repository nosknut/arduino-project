#ifndef MainRosserialBridge_h
#define MainRosserialBridge_h
#include <Arduino.h>
#include <RosserialPublisherBridge.h>

class MainRosserialBridge
{
private:
    // Set the rosserial socket server IP address
    IPAddress server(192, 168, 1, 1);

    ros::NodeHandle nh;

    RosPublisherBridge<sensor_msgs::Imu> imuBridge("imu");
    RosPublisherBridge<std_msgs::Int16> leftEncoderBridge("left_ticks");
    RosPublisherBridge<std_msgs::Int16> rightEncoderBridge("right_ticks");
    RosPublisherBridge<sensor_msgs::Range> irBridge("range_data");

public:
    void setup(uint16_t serverPort = 11411)
    {
        // Set the connection to rosserial socket server
        nh.getHardware()->setConnection(server, serverPort);
        nh.initNode();

        imuBridge.setup();
        leftEncoderBridge.setup();
        rightEncoderBridge.setup();
        irBridge.setup();
    }

    void loop()
    {

        if (nh.connected())
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
        nh.spinOnce();
    }
};

#endif
