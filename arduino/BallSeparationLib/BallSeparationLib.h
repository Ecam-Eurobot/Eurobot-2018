  /**********************************************************/
  /*!
	Ball separating library used to hide lower level instruc-
	tions from the main file.
  */
  /**********************************************************/

#include <Adafruit_TCS34725.h>
#include <Servo.h>

//Functions
int     csStartUpRoutine(void);
const char*     csCheckTeamColor(void);
void     servoComputePos(int position);
void     servoStart(void);