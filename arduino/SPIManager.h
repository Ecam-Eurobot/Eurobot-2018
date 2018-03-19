#ifndef _SPIMANAGER_H_
#define _SPIMANAGER_H_

#include <SPI.h>


class SPIManager {
public:
    SPIManager(uint8_t slaveSelectPin);
    boolean initialize();
    float readData(byte thisRegister);
    long readLongData(byte thisRegister);
    void writeData(byte thisRegister, byte msgSize, byte thisValue[]);
    void end();
protected:
    uint8_t _slaveSelectPin;
};

#endif
