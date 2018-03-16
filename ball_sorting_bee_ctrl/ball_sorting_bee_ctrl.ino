#include <ros.h>
#include <std_msgs/Bool.h>
#include <BallSeparationLib.h>

#define ID1    (7u)
#define ID2    (1u)

//Dynamixel AX-12A definitions
const int servo_speed = 500; //speed for ax-12a movement

// Bool variable decides whether to push bee
// or return to initial position. False = initial position
// True = push bee position 
bool pushbee = false;
int pushbee_pos = 180;   // push position for bee
int initialpos = 500;

//Start ROS handle.
ros::NodeHandle nh;

//std_msgs::Float32 str_msg;
void moveBee( const std_msgs::Bool & position_msg){
  pushbee = position_msg.data;
  if(pushbee){
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(300);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(300);                       // wait for a second
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(300);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(300);                       // wait for a second
    ax12Movedebug(ID2, 300, servo_speed);
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(300);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(300);                       // wait for a second
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(300);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(300);                       // wait for a second 
    ax12Movedebug(ID1, 250, servo_speed);
    }
  else {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(300);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(300);                       // wait for a second
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(300);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    ax12Movedebug(ID2, 50, servo_speed);
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(300);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(300);                       // wait for a second
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(300);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(300);                       // wait for a second
    ax12Movedebug(ID1, 450, servo_speed);
    }
}
ros::Subscriber<std_msgs::Bool> bee_ctrl("bee_control", &moveBee);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  nh.initNode();
  nh.subscribe(bee_ctrl);
  ax12Start(servo_speed);
}


//the loop contains is empty.
void loop() {
  nh.spinOnce();
}
