#ifndef WIRE_H_STUB
#define WIRE_H_STUB
#include <Arduino.h>
class TwoWireStub {
public:
    bool begin(int sda, int scl, uint32_t freq);
    void beginTransmission(uint8_t address);
    uint8_t endTransmission(void);
};
extern TwoWireStub Wire;
#endif
