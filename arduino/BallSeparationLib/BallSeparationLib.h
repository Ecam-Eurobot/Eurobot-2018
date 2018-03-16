  /**********************************************************/
  /*!
	Ball separating library used to hide lower level instruc-
	tions from the main file.
  */
  /**********************************************************/

#include <Adafruit_TCS34725.h>
#include <Servo.h>
#include <AX12A.h>

#define DirectionPin  (10u)
#define BaudRate    (1000000ul)
#define ID    (8u)

//Functions
int     csStartUpRoutine(void);
const char*     csCheckTeamColor(void);
void     servoComputePos(int position);
void     servoStart(void);
void     ax12Start(int speed);
void     ax12ComputePos(unsigned char id, byte position, int speed);
void     ax12Blink(void);
void     ax12Movedebug(unsigned char id, int position, int speed);