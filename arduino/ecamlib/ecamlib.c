#include "ecamlib.h"

float bytesToFloat(unsigned char b0, unsigned char b1, unsigned char b2, unsigned char b3) {
    union {
        float f;
        unsigned char b[4];
    } number;

    number.b[0] = b0;
    number.b[1] = b1;
    number.b[2] = b2;
    number.b[3] = b3;

    return number.f;
}

int bytesToInt(unsigned char b0, unsigned char b1) {
    union {
        int i;
        unsigned char b[2];
    } number;

    number.b[0] = b0;
    number.b[1] = b1;

    return number.i;
}

long bytesToLong(unsigned char b0, unsigned char b1, unsigned char b2, unsigned char b3) {
    union {
        long l;
        unsigned char b[4];
    } number;

    number.b[0] = b0;
    number.b[1] = b1;
    number.b[2] = b2;
    number.b[3] = b3;

    return number.l;
}