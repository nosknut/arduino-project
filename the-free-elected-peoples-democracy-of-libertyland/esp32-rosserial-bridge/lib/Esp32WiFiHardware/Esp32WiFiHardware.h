#ifndef Esp32_WiFi_Hardware_h
#define Esp32_WiFi_Hardware_h
#include <WiFi.h>

class Esp32WiFiHardware
{
public:
    Esp32WiFiHardware()
    {
    }

    void setConnection(IPAddress &server, int port)
    {
        this->server = server;
        this->serverPort = port;
    }

    IPAddress getLocalIP()
    {
        return tcp.localIP();
    }

    void init()
    {
        this->tcp.connect(this->server, this->serverPort);
    }

    int read()
    {
        if (this->tcp.connected())
        {
            return tcp.read();
        }
        else
        {
            this->tcp.connect(this->server, this->serverPort);
        }
        return -1;
    };

    void write(const uint8_t *data, size_t length)
    {
        tcp.write(data, length);
    }

    unsigned long time() { return millis(); }

protected:
    WiFiClient tcp;
    IPAddress server;
    uint16_t serverPort = 11411;
};

#endif
