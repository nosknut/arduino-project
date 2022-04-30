#ifndef RosserialBridge_h
#define RosserialBridge_h
#include <Arduino.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
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
class RosserialBridge
{
protected:
    MessageType msg;

private:
    // TODO: Test if this is assigned before pub and sub are instantiated
    const String nodeName;

    ros::Publisher pub = ros::Publisher(nodeName.c_str(), &msg);

    // void publish(const MessageType &cmd_msg)
    //{
    //     pub.publish(&cmd_msg);
    // }

    // Set the publiser.publish as the subscriber event handler
    // https://stackoverflow.com/questions/68550716/pass-non-static-class-member-callback-function-to-rossubscriber
    // SerialSubscriber<MessageType, RosserialBridge> sub =
    //    SerialSubscriber<MessageType, RosserialBridge>(nodeName.c_str(), &RosserialBridge::publish, this);

public:
    RosserialBridge(String nodeName) : nodeName(nodeName)
    {
        static_assert(std::is_base_of<ros::Msg, MessageType>::value, "MessageType type parameter of this class must derive from ros::Msg");
    }

    void setup(OutputNodeHandle &outputNh)
    {
        outputNh.advertise(pub);
    }

    virtual void mapMessages(OutputNodeHandle &nh, JsonDocument &inDoc, MessageType &outMsg) = 0;

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
