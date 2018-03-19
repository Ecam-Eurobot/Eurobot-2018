#ifndef ecamlib_h
#define ecamlib_h

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
  #include "pins_arduino.h"
  #include "WConstants.h"
#endif

/*
    C++ performs name mangling, in order to be able to
    call C functions from it we need to tell it to use 
    the C convention for these functions.
*/
#ifdef __cplusplus
extern "C" {
#endif

    /*
        bytesToFloat expects an array of four bytes and
        converts them to a float value
    */
    float bytesToFloat(unsigned char b0, unsigned char b1, unsigned char b2, unsigned char b3);

    /*
        bytesToInt expects an array of two bytes and
        converts them to a int value. Because on the
        Arduino Uno ints are 16-bits.
    */
    int bytesToInt(unsigned char b0, unsigned char b1);

    /*
        bytesToLong expects an array of four bytes and
        converts them to a long value. Because on the
        Arduino Uno longs are 32-bits.
    */
    long bytesToLong(unsigned char b0, unsigned char b1, unsigned char b2, unsigned char b3);


#ifdef __cplusplus
}
#endif

#endif