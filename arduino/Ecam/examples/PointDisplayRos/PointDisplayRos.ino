#include <std_msgs/Int64.h>
#include <ros.h>
ros::Subscriber<std_msgs::Int64> point_dis("PointDisplay" , &point);

int number;
void point( const std_msgs::Int64 & toggle_msg){
  number = toggle_msg.data;
}


#include "Wire.h" // enable I2C bus
byte saa1064 = 0x70 >> 1; // define the I2C bus address for our SAA1064 (pin 1 to GND) ****
int digits[16]={63, 6, 91, 79, 102, 109, 125,7, 127, 111, 119, 124, 57, 94, 121, 113};
// these are the byte representations of pins required to display each digit 0~9 then A~F
void setup()
{
 nh.initNode();
  nh.subscribe(point_dis); 
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

void displayDigits(int number)
// show all digits 0~9, A~F on all digits of display
{
 int U = number%10;
 int D = (number%100 - U)/10;
 int C = (number%1000 - D)/100;
 int M = number/1000;
 Wire.beginTransmission(saa1064);
 Wire.write(1); // instruction byte - first digit to control is 1 (right hand side)
 Wire.write(digits[U]); 
 Wire.write(digits[C]);
 Wire.write(digits[D]); 
 Wire.write(digits[M]); 
 Wire.endTransmission();
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
 displayDigits(number);
 //clearDisplay();
 delay(1000);
}

