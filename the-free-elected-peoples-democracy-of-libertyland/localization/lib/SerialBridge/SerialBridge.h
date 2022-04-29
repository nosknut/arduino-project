#ifndef SerialBridge_h
#define SerialBridge_h
#include <Arduino.h>
#include <ros/time.h>
#include <SerialClass.h>

#ifndef TERMINATING_CHARACTER
#define TERMINATING_CHARACTER ';'
#endif
#ifndef STARTING_CHARACTER
#define STARTING_CHARACTER ':'
#endif
#ifndef MESSAGE_TERMINATNG_CHARACTER
#define MESSAGE_TERMINATNG_CHARACTER '`'
#endif

class SerialConnection
{
private:
    SerialClass *iostream;
    int numLoopsWithoutData = 0;
    bool isReadingMessage = false;
    String newestTopic = "";

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
        iostream->write(STARTING_CHARACTER);
        iostream->print(topic);
        iostream->write(TERMINATING_CHARACTER);
        iostream->write((const char *)message, sizeof(MessageType));
        Serial.println(sizeof(MessageType));
        iostream->write(MESSAGE_TERMINATNG_CHARACTER);
    }

    template <typename MessageType>
    bool readMessage(String topic, MessageType &message)
    {
        if (!isReadingMessage)
        {
            if (iostream->available())
            {
                if (iostream->read() == STARTING_CHARACTER)
                {
                    newestTopic = iostream->readStringUntil(TERMINATING_CHARACTER);
                    isReadingMessage = true;
                }
            }
        }
        if (isReadingMessage)
        {
            // https://www.cplusplus.com/reference/cstring/strcmp/
            if (topic == newestTopic)
            {
                iostream->readBytesUntil(MESSAGE_TERMINATNG_CHARACTER, (char *)&message, sizeof(MessageType));
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
        if (isReadingMessage)
        {
            numLoopsWithoutData++;
        }
        if (numLoopsWithoutData > 2)
        {
            isReadingMessage = false;
            numLoopsWithoutData = 0;
        }
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
