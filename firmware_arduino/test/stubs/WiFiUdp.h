#ifndef WIFIUDP_H_STUB
#define WIFIUDP_H_STUB
#include <WiFi.h>

class WiFiUDP {
public:
    uint8_t begin(uint16_t port);
    int parsePacket(void);
    int read(uint8_t *buf, size_t len);
    IPAddress remoteIP(void);
    int beginPacket(IPAddress ip, uint16_t port);
    size_t write(const uint8_t *buf, size_t len);
    int endPacket(void);
};
#endif
