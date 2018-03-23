#include <SPIManager.h>

SPIManager::SPIManager(uint8_t slaveSelectPin) {
    _slaveSelectPin = slaveSelectPin;
}

/// setup SPI, read device ID etc...
boolean SPIManager::initialize()
{
    pinMode(_slaveSelectPin, OUTPUT);
    digitalWrite(_slaveSelectPin, HIGH);
    SPI.begin();
    return true;
}

//Read from or write to register
float SPIManager::readData(byte thisRegister) {
    union {
        float f;
        unsigned char b[4];
    } u;
    digitalWrite(_slaveSelectPin, LOW);
    delayMicroseconds(20);
    SPI.transfer(thisRegister);
    delayMicroseconds(20);
    SPI.transfer(0x04);
    for( int i = 0; i < 4; i = i + 1 ) {
        delayMicroseconds(20);
        u.b[i] = SPI.transfer(0x00);
    }
    digitalWrite(_slaveSelectPin, HIGH);
    delayMicroseconds(2000);
    return (u.f);
}

long SPIManager::readLongData(byte thisRegister,byte msgSize, byte thisValue[]) {
    union {
        unsigned long l;
        unsigned char b[4];
    } u;
    digitalWrite(_slaveSelectPin, LOW);
    delayMicroseconds(20);
    SPI.transfer(thisRegister);
    delayMicroseconds(20);
    SPI.transfer(msgSize);
    for( int i = 0; i < 4; i = i + 1 ) {
        delayMicroseconds(20);
        u.b[i] = SPI.transfer(thisValue[i]);
    }
    digitalWrite(_slaveSelectPin, HIGH);
    delayMicroseconds(2000);
    return (u.l);
}


//Sends a write command
void SPIManager::writeData(byte thisRegister, byte msgSize, byte thisValue[]) {
    // take the chip select low to select the device:
    digitalWrite(_slaveSelectPin, LOW);
    delayMicroseconds(20);
    SPI.transfer(thisRegister); //Send register location
    delayMicroseconds(20);
    SPI.transfer(msgSize); //Send size of msg
    for( int i = 0; i < msgSize; i = i + 1 ) {
        delayMicroseconds(20);
        SPI.transfer(thisValue[i]);//Send value to record into register
    }
    // take the chip select high to de-select:
    digitalWrite(_slaveSelectPin, HIGH);
    delayMicroseconds(2000);
}

/// cleanup
void SPIManager::end() {
    SPI.end();
}
