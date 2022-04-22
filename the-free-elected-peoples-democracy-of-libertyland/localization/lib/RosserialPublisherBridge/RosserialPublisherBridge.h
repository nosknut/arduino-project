#ifndef RosserialPublisherBridge_h
#define RosserialPublisherBridge_h
#include <Arduino.h>
#include <ros.h>

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
template <typename MessageType>
class RosserialPublisherBridge
{
private:
    // TODO: Test if this is assigned before pub and sub are instantiated
    const String nodeName;

    MessageType msg;
    ros::Publisher pub(nodeName, &msg);
    // Set the publiser.publish as the subscriber event handler
    ros::Subscriber<MessageType> sub(nodeName, pub.publish);

public:
    RosPublisherBridge(String nodeName) : nodeName(nodeName)
    {
        static_assert(std::is_base_of<ros::Msg, MessageType>::value, "MessageType type parameter of this class must derive from ros::Msg");
    }

    void setup(ros::NodeHandle &nh)
    {
        nh.advertise(pub_imu);
        nh.subscribe(sub);
    }
};

#endif
