/*
  Morse.h - Library for flashing Morse code.
  Created by David A. Mellis, November 2, 2007.
  Released into the public domain.
*/
#ifndef SPIslave_h
#define SPIslave_h
#include "Arduino.h"

class SpiSlave{
public:
    SpiSlave();
    void begin();
    void reset();
    void com(byte data);
    bool endTrans = false;
    byte dataSize = 0x00;
    byte command = 0x00;
    byte msg[8];
    int dataCount = 0;
};
#endif
