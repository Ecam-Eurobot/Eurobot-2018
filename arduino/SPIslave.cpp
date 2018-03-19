#include "Arduino.h"
#include "SPIslave.h"

SpiSlave::SpiSlave() {}

void SpiSlave::begin(){
    pinMode(MISO, OUTPUT);
    //SPCR = SPI Control Register
    // turn on SPI in slave mode
    SPCR |= _BV(SPE);
    // turn on interrupts
    SPCR |= _BV(SPIE);

}

void SpiSlave::reset() {
        dataCount = 0;
}


void SpiSlave::com(byte data) {
  switch (dataCount){
      case 0 :
          command = data;
          break;
      case 1 :
          dataSize = data;
          break;
      default:
          msg[dataCount-2] = data;
  }
    dataCount++;
    if (dataSize == dataCount-1){
      endTrans = true;
    }
    else {
      endTrans = false;
    }
}

