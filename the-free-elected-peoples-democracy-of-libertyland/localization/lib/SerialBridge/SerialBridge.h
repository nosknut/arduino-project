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

template <typename MessageType>
class SerialPublisher;

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
    void readMessage(char *topic[TOPIC_NAME_SIZE], MessageType &message)
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
                        return;
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
            }
        }
    }

    template <typename MessageType>
    void advertise(SerialPublisher<MessageType> &serialPublisher)
    {
    }

    void initNode()
    {
    }

    SerialConnection *getHardware()
    {
        return this;
    }

    void spinOnce()
    {
    }
};

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

/* Base class for objects subscribers. */
class SerialSubscriber_
{
public:
    virtual void callback(unsigned char *data) = 0;
    const char *topic_;
};

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
    };

    virtual void callback(unsigned char *data)
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
    };

    SerialSubscriber(const __FlashStringHelper *topic_name, CallbackT cb) : cb_(cb)
    {
        topic_ = reinterpret_cast<const char *>(topic_name);
    };

    virtual void callback(unsigned char *data)
    {
        this->cb_(msg);
    }

private:
    CallbackT cb_;
};

typedef SerialConnection NodeHandleClass;
#define PublisherClass SerialPublisher
#define SubscriberClass SerialSubscriber

#endif
