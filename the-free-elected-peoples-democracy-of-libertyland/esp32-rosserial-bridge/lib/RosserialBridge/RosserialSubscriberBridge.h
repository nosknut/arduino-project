#ifndef RosserialSubscriberBridge_h
#define RosserialSubscriberBridge_h
#include <Arduino.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ArduinoJson.h>

/**
 * A generic class that creates a sub-pub proxy
 * Useful when you have a different device generating
 * messages. This class should be run on the device
 * that sits between the original publisher and the
 * intended subscriber.
 *
 * Example:
 * Publisher: Arduino uno
 * Bridge: Esp32
 * Subscriber: Raspberry Pi
 *
 * The Arduino Uno and Esp32 would be connected
 * via a serial port.
 *
 * The Esp32 and Raspberry Pi would be connected
 * over WiFi.
 *
 */

template <typename MessageType, typename OutputNodeHandle>
class RosserialSubscriberBridge
{
protected:
    MessageType msg;

    virtual void mapMessages(OutputNodeHandle &nh, JsonDocument &inDoc, MessageType &outMsg) = 0;

private:
    // TODO: Test if this is assigned before pub and sub are instantiated
    const String nodeName;

    ros::Publisher pub = ros::Publisher(nodeName.c_str(), &msg);

public:
    RosserialSubscriberBridge(String nodeName) : nodeName(nodeName)
    {
        static_assert(std::is_base_of<ros::Msg, MessageType>::value, "MessageType type parameter of this class must derive from ros::Msg");
    }

    void setup(OutputNodeHandle &outputNh)
    {
        outputNh.advertise(pub);
    }

    void loop(JsonDocument &inputDoc, OutputNodeHandle &outputNh)
    {
        if (inputDoc["topic"] == nodeName)
        {
            mapMessages(outputNh, inputDoc, msg);
            pub.publish(&msg);
        }
    }
};

#endif
