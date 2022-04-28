#ifndef SerialBridge_h
#define SerialBridge_h
#include <Arduino.h>
#include <ArduinoIncludes.h>

#ifndef TOPIC_NAME_SIZE
#define TOPIC_NAME_SIZE 20
#endif

#ifndef TERMINATING_CHARACTER
#define TERMINATING_CHARACTER ';'
#endif

#ifndef MAX_SERIAL_SUBSCRIBERS
#define MAX_SERIAL_SUBSCRIBERS 20
#endif

template <typename MessageType>
class SerialPublisher;

/* Base class for objects subscribers. */
class SerialSubscriber_
{
public:
    virtual void callback(unsigned char *data) = 0;
    const char *topic_;
    int id_;
    int messageSize_;
};

class SerialConnection
{
private:
    SERIAL_CLASS *iostream;
    bool isReadingMessage = false;
    char newestTopic[TOPIC_NAME_SIZE];

public:
    SerialConnection(SERIAL_CLASS *iostream)
    {
        this->iostream = iostream;
    }

    void setBaud(unsigned long baud)
    {
        iostream->begin(baud);
    }

    void setTimeout(unsigned long timeout)
    {
        iostream->setTimeout(timeout);
    }

    template <typename MessageType>
    void publish(char *topic[TOPIC_NAME_SIZE], MessageType &message)
    {
        Serial.print(String(topic));
        Serial.print(message);
        Serial.print(TERMINATING_CHARACTER);
    }

    template <typename MessageType>
    bool readMessage(char *topic[TOPIC_NAME_SIZE], MessageType &message)
    {
        if (!isReadingMessage)
        {
            if (Serial.available() >= TOPIC_NAME_SIZE)
            {
                for (int i = 0; i < TOPIC_NAME_SIZE; i++)
                {
                    char value = Serial.read();
                    // Exit if we see the terminating character
                    // It will be treated as a hard reset, in case the library gets out of sync
                    if (value == TERMINATING_CHARACTER)
                    {
                        isReadingMessage = false;
                        return false;
                    }
                    newestTopic[i] = value;
                }
                isReadingMessage = true;
            }
        }
        if (isReadingMessage)
        {
            if (String(topic) == String(newestTopic))
            {
                uint8_t *buffer;
                Serial.readBytes(buffer, sizeof(message));
                message = *(MessageType *)buffer;
                isReadingMessage = false;
                return true;
            }
        }
        return false;
    }
};

template <typename Hardware>
class SerialNodeHandle_
{
private:
    Hardware *hardware;
    int subscriberCount = 0;
    SerialSubscriber_ *subscribers[MAX_SERIAL_SUBSCRIBERS];

    SerialNodeHandle_(Hardware *hardware) : hardware(hardware)
    {
    }

    template <typename MessageType>
    void advertise(SerialPublisher<MessageType> &serialPublisher)
    {
    }

    /* Register a new subscriber */
    template <typename SubscriberT>
    bool subscribe(SubscriberT &s)
    {
        if (subscriberCount < MAX_SERIAL_SUBSCRIBERS)
        {
            subscribers[subscriberCount] = static_cast<SerialSubscriber_ *>(&s);
            s.id_ = subscriberCount;
            subscriberCount++;
            return true;
        }
        return false;
    }

    void initNode()
    {
    }

    Hardware *getHardware()
    {
        return hardware;
    }

    void spinOnce()
    {
        for (int i = 0; i < subscriberCount; i++)
        {
            SerialSubscriber_ *subscriber = subscribers[i];
            if (subscriber->topic_ != nullptr)
            {
                char message[subscriber->messageSize_];
                if (getHardware()->readMessage(subscriber->topic_, message))
                {
                    (subscriber->*(subscriber->callback))(message);
                }
            }
        }
    }
};

typedef SerialNodeHandle_<SerialConnection> SerialNodeHandle;

template <typename MessageType>
class SerialPublisher
{
private:
    SerialConnection *serialConnection;
    char topic[TOPIC_NAME_SIZE];

public:
    SerialPublisher(char topic[TOPIC_NAME_SIZE], SerialConnection *serialConnection)
    {
        this->serialConnection = serialConnection;
        strcpy(this->topic, topic);
    }

    void publish(MessageType &message)
    {
        serialConnection->publish(topic, message);
    }

    void readMessage(MessageType &message)
    {
        serialConnection->readMessage(topic, message);
    }
};

/*
template <typename MessageType>
class SerialSubscriber
{
private:
    typedef void (ObjT::*CallbackT)(const MsgT &);
    SerialConnection *serialConnection;
    char topic[TOPIC_NAME_SIZE];

public:
    SerialSubscriber(char topic[TOPIC_NAME_SIZE], SerialConnection *serialConnection)
    {
        this->serialConnection = serialConnection;
        strcpy(this->topic, topic);
    }

    void publish(MessageType &message)
    {
        serialConnection->publish(topic, message);
    }

    void readMessage(MessageType &message)
    {
        serialConnection->readMessage(topic, message);
    }
};
*/

/* Bound function subscriber. */
template <typename MsgT, typename ObjT = void>
class SerialSubscriber : public SerialSubscriber_
{
public:
    typedef void (ObjT::*CallbackT)(const MsgT &);
    MsgT msg;

    SerialSubscriber(const char *topic_name, CallbackT cb, ObjT *obj) : cb_(cb), obj_(obj)
    {
        topic_ = topic_name;
        messageSize_ = sizeof(MsgT);
    };

    void callback(unsigned char *data)
    {
        (obj_->*cb_)(msg);
    }

private:
    CallbackT cb_;
    ObjT *obj_;
};

/* Standalone function subscriber. */
template <typename MsgT>
class SerialSubscriber<MsgT, void> : public SerialSubscriber_
{
public:
    typedef void (*CallbackT)(const MsgT &);
    MsgT msg;

    SerialSubscriber(const char *topic_name, CallbackT cb) : cb_(cb)
    {
        topic_ = topic_name;
        messageSize_ = sizeof(MsgT);
    };

    SerialSubscriber(const __FlashStringHelper *topic_name, CallbackT cb) : cb_(cb)
    {
        topic_ = reinterpret_cast<const char *>(topic_name);
        messageSize_ = sizeof(MsgT);
    };

    void callback(unsigned char *data)
    {
        this->cb_(msg);
    }

private:
    CallbackT cb_;
};

#endif
