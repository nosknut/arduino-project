#ifndef SerialBridge_h
#define SerialBridge_h
#include <Arduino.h>
#include <ros/time.h>
#include <SerialClass.h>

#ifndef TERMINATING_CHARACTER
#define TERMINATING_CHARACTER ';'
#endif
#ifndef TOPIC_NAME_SIZE
#define TOPIC_NAME_SIZE 20
#endif

class SerialConnection
{
private:
    SerialClass *iostream;
    bool isReadingMessage = false;
    char newestTopic[TOPIC_NAME_SIZE];

public:
    SerialConnection(SerialClass *iostream)
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
    void publish(String topic, MessageType *message)
    {
        char topicName[TOPIC_NAME_SIZE];
        topic.toCharArray(topicName, TOPIC_NAME_SIZE);
        iostream->print(topicName);
        iostream->write((const char *)message, sizeof(MessageType));
        iostream->print(TERMINATING_CHARACTER);
    }

    template <typename MessageType>
    bool readMessage(String topic, MessageType &message)
    {
        if (!isReadingMessage)
        {
            if (iostream->available() >= TOPIC_NAME_SIZE)
            {
                for (int i = 0; i < TOPIC_NAME_SIZE; i++)
                {
                    char value = iostream->read();
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
            if (topic == String(newestTopic))
            {
                uint8_t *buffer;
                iostream->readBytes(buffer, sizeof(message));
                message = *(MessageType *)buffer;
                isReadingMessage = false;
                return true;
            }
        }
        return false;
    }

    void initNode()
    {
    }

    SerialConnection *getHardware()
    {
        return this;
    }

    bool connected()
    {
        return true;
    }

    void spinOnce()
    {
    }

    unsigned long time()
    {
        return millis();
    }

    ros::Time now()
    {
        uint32_t ms = time();
        ros::Time current_time;
        current_time.sec = ms / 1000;
        current_time.nsec = (ms % 1000) * 1000000UL;
        return current_time;
    }
};

#endif
