/*
 Example 39.1 - NXP SAA1064 I2C LED Driver IC Demo I
 Demonstrating display of digits
 http://tronixstuff.com/tutorials > chapter 39
 John Boxall July 2011 | CC by-sa-nc
 */
#include "Wire.h" // enable I2C bus
byte saa1064 = 0x70 >> 1; // define the I2C bus address for our SAA1064 (pin 1 to GND) ****
int digits[16]={63, 6, 91, 79, 102, 109, 125,7, 127, 111, 119, 124, 57, 94, 121, 113};
// these are the byte representations of pins required to display each digit 0~9 then A~F
void setup()
{
 Wire.begin(); // start up I2C bus
 delay(500);
 initDisplay();
}
void initDisplay()
// turns on dynamic mode and adjusts segment current to 12mA
{
 Wire.beginTransmission(saa1064);
 Wire.write(B00000000); // this is the instruction byte. Zero means the next byte is the control byte
 Wire.write(B01000111); // control byte (dynamic mode on, digits 1+3 on, digits 2+4 on, 12mA segment current
 Wire.endTransmission();
}
void displayDigits()
// show all digits 0~9, A~F on all digits of display
{
 for (int z=0; z<16; z++)
 {
 Wire.beginTransmission(saa1064);
 Wire.write(1); // instruction byte - first digit to control is 1 (right hand side)
 Wire.write(digits[z]); // digit 1 (RHS)
 Wire.write(digits[z]); // digit 2
 Wire.write(digits[z]); // digit 3
 Wire.write(digits[z]); // digit 4 (LHS)
 Wire.endTransmission();
 delay(500);
 }
// now repeat but with decimal point
 for (int z=0; z<16; z++)
 {
 Wire.beginTransmission(saa1064);
 Wire.write(1); // instruction byte - first digit to control is 1 (right hand side)
 Wire.write(digits[z]+128); // digit 1 (RHS)
 Wire.write(digits[z]+128); // digit 2
 Wire.write(digits[z]+128); // digit 3
 Wire.write(digits[z]+128); // digit 4 (LHS)
 Wire.endTransmission();
 delay(500);
 }
}
void clearDisplay()
// clears all digits
{
 Wire.beginTransmission(saa1064);
 Wire.write(1); // instruction byte - first digit to control is 1 (right hand side)
 Wire.write(0); // digit 1 (RHS)
 Wire.write(0); // digit 2
 Wire.write(0); // digit 3
 Wire.write(0); // digit 4 (LHS)
 Wire.endTransmission();
}
void loop()
{
 displayDigits();
 clearDisplay();
 delay(1000);
}
/* **** We bitshift the address as the SAA1064 doesn't have the address 0x70 (ADR pin
to GND) but 0x38 and Arduino uses 7-bit addresses- so 8-bit addresses have to
be shifted to the right one bit. Thanks to Malcolm Cox */
