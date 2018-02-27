/**************************************************************************/
/*!
    @file     BallSeparationLib.cpp
    @author   Ecambotics
    @license  opensource

    Abstraction for lower level instructions.

    v1.0 - First release
*/
/**************************************************************************/
#include <stdlib.h>
#include "BallSeparationLib.h"

// Create cs object and Initialise with specific values
// (int time = 50ms, gain = 4x) 
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

//Servo definitions
Servo servo;
int servoPin = 9;   
int servoAngle = 0;   // servo position in degrees

/**************************************************************************/
/*!
    @brief  Starts up CS. Returns 1 if successfully completed, 0 for any 
			malfunction
*/
/**************************************************************************/
int csStartUpRoutine(void)
{
	int result = 0;
	if (tcs.begin()) {
		result = 1;
		return result;
	} else {
		return result;
		//while (1); // halt!
	}	
}

/**************************************************************************/
/*!
    @brief  
*/
/**************************************************************************/
const char* csCheckTeamColor(void)
{
	//Variables to hold RGB values of colour detected
	uint16_t clear, red, green, blue;

	//Lower and upper ranges for green and orange
	uint16_t upperrangeG[3] = {100, 153, 71};
	uint16_t lowerrangeG[3] = {50, 104, 52};
	uint16_t upperrangeO[3] = {216,75,44};
	uint16_t lowerrangeO[3] = {155, 60, 32};  
	float r, g, b;
	const char* teamColour;

	tcs.setInterrupt(false);      // turn on LED
	delay(60);                    // takes 50ms to read                  
	tcs.getRawData(&red, &green, &blue, &clear);               
	tcs.setInterrupt(true);      // turn off LED

	// Figure out some basic hex code for visualization
	uint32_t sum = clear;
	r = red; 
	r /= sum;
	g = green; 
	g /= sum;
	b = blue; 
	b /= sum;
	r *= 256; g *= 256; b *= 256;
	Serial.print("\t");
	

	Serial.print((int)r, HEX); Serial.print((int)g, HEX); Serial.print((int)b, HEX);
	Serial.println();
	Serial.print((int)r ); Serial.print(" "); Serial.print((int)g);Serial.print(" ");  Serial.println((int)b );

	if((r < upperrangeG[0] && g < upperrangeG[1] && b < upperrangeG[2]) || (r > lowerrangeG[0] && g > lowerrangeG[1] && b > lowerrangeG[2])){
	  delay(100);
	  teamColour = "Green";
	  //Serial.print("Green");
	}
	if((r < upperrangeO[0] && g < upperrangeO[1] && b < upperrangeO[2]) || (r > lowerrangeO[0] && g > lowerrangeO[1] && b > lowerrangeO[2])){
	  delay(100);
	  teamColour= "Orange";
	  //Serial.print("Orange");
	}
	
	return teamColour;
}

void servoStart(void)
{
	servo.attach(servoPin);
}

void servoComputePos(int position)
{
	if(position == 1){		
		//Clean ball collection
		servoAngle = 90;
		servo.write(servoAngle);
	}
	if(position == 2){
		//Dirty ball collection
		servoAngle = 0;
		servo.write(servoAngle);
	} 
}