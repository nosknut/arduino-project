#ifndef RosserialBridge_h
#define RosserialBridge_h
#include <Arduino.h>
#include <ros.h>

/**
 * TODO: Make sure this hack works
 * Handle for AVR_ATmega328P
 * MAX_SUBSCRIBERS=25
 * MAX_PUBLISHERS=25
 * INPUT_SIZE=280
 * OUTPUT_SIZE=280
 */
typedef ros::NodeHandle_<ArduinoHardware, 25, 25, 280, 280, ros::FlashReadOutBuffer_> AVR_ATmega328P_NodeHandle;

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
class RosserialBridge
{
private:
    // TODO: Test if this is assigned before pub and sub are instantiated
    const String nodeName;

    MessageType msg;
    ros::Publisher pub = ros::Publisher(nodeName.c_str(), &msg);

    void publish(const MessageType &cmd_msg)
    {
        pub.publish(&cmd_msg);
    }

    // Set the publiser.publish as the subscriber event handler
    // https://stackoverflow.com/questions/68550716/pass-non-static-class-member-callback-function-to-rossubscriber
    ros::Subscriber<MessageType, RosserialBridge> sub =
        ros::Subscriber<MessageType, RosserialBridge>(nodeName.c_str(), &RosserialBridge::publish, this);

public:
    RosserialBridge(String nodeName) : nodeName(nodeName)
    {
        static_assert(std::is_base_of<ros::Msg, MessageType>::value, "MessageType type parameter of this class must derive from ros::Msg");
    }

    void setup(AVR_ATmega328P_NodeHandle &inputNh, ros::NodeHandle &outputNh)
    {
        outputNh.advertise(pub);
        inputNh.subscribe(sub);
    }
};

#endif
