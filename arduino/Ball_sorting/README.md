# Ball_sorting

## Main Code

This code currently implements I2C communication on an arduino board 
allowing for messages from a master controller to pilot the Adafruit_TCS34725
colour sensor for colour detection and to position a servo motor depending on 
data sent over bus. 

## BallSeparationLib

`BallSeparationLib` is an Arduino library containing high level functions
used to hide lower level instructions from main and thus ease readability. 
Currently, it contains functions to identify the team's colour for the robot
and direct control over the position of a servo motor.

### Dependencies

**Servo.h**		Already installed in Arduino IDE

**ecamlib.h**	Cross reference installation guide/readme for ecamlib	

**Adafruit_TCS34725.h**	Colour sensor. Needs to be dowloaded and installed as described here: 
						https://learn.adafruit.com/adafruit-color-sensors/arduino-code
						
**AX12A.h**	Dynamixel AX-12A library. Can be downloaded and installed here: https://github.com/ThingType/AX-12A-servo-library	

### Install

Cross-reference installation guide of **ecamlib.h**, `BallSeparationLib` can be
installed in the same way either by **Zip**, **Copy**, or **SymLink**.

 
### Usage
When the library is installed in the Arduino IDE, you should be
able to use it by selecting `Sketch > Include Library > BallSeparationLib`
You will probably need to scroll to the end of the list.

It will add an include in the source file for you:

```cpp
#include <BallSeparationLib.h>
```