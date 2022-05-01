#ifndef RosserialPublisherBridge_h
#define RosserialPublisherBridge_h
#include <Arduino.h>
#include <SerialClass.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ArduinoJson.h>

/**
 * A generic class that creates a pub-sub proxy
 * Useful when you have a different device generating
 * messages. This class should be run on the device
 * that sits between the original publisher and the
 * intended subscriber.
 *
 * Example:
 * Publisher: Raspberry Pi
 * Bridge: Esp32
 * Subscriber: Arduino uno
 *
 * The Arduino Uno and Esp32 would be connected
 * via a serial port.
 *
 * The Esp32 and Raspberry Pi would be connected
 * over WiFi.
 *
 */

template <typename MessageType, typename InputNodeHandle>
class RosserialPublisherBridge
{
protected:
    DynamicJsonDocument outputDocument = DynamicJsonDocument(200);

    virtual void mapMessages(const MessageType &inMsg, JsonDocument &outDoc) = 0;

private:
    // TODO: Test if this is assigned before pub and sub are instantiated
    const String nodeName;

    void onMessage(const MessageType &cmd_msg)
    {
        mapMessages(cmd_msg, outputDocument);
        serializeJson(outputDocument, DATA_SERIAL_CLASS);
    }

    // https://stackoverflow.com/questions/68550716/pass-non-static-class-member-callback-function-to-rossubscriber
    ros::Subscriber<MessageType, RosserialPublisherBridge> sub =
        ros::Subscriber<MessageType, RosserialPublisherBridge>(nodeName.c_str(), &RosserialPublisherBridge::onMessage, this);

public:
    RosserialPublisherBridge(String nodeName) : nodeName(nodeName)
    {
        static_assert(std::is_base_of<ros::Msg, MessageType>::value, "MessageType type parameter of this class must derive from ros::Msg");
    }

    void setup(InputNodeHandle &inputNh)
    {
        inputNh.subscribe(sub);
    }
};

#endif
