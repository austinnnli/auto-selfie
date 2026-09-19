#ifndef WIFI_H_STUB
#define WIFI_H_STUB
#include <Arduino.h>

#define WL_CONNECTED 3
#define WIFI_STA 1

class IPAddress {
public:
    IPAddress() {}
    IPAddress(uint32_t a) { (void)a; }
};

class WiFiStub {
public:
    void persistent(bool p);
    void mode(int m);
    void setAutoReconnect(bool r);
    void setSleep(bool s);
    void begin(const char *ssid, const char *pass);
    int status(void);
    IPAddress localIP(void);
};
extern WiFiStub WiFi;
#endif
