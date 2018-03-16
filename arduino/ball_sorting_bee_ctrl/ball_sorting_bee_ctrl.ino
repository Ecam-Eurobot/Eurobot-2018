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
int pushbee_pos = 180;   // initial/push position for bee still needs to be calibrated
int initialpos = 500;

//Start ROS handle.
ros::NodeHandle nh;

//std_msgs::Float32 str_msg;
void moveBee( const std_msgs::Bool & position_msg){
  pushbee = position_msg.data;
  if(pushbee){
    ax12Movedebug(ID2, pushbee_pos, servo_speed);
    }
  else {
    ax12Movedebug(ID2, initialpos, servo_speed);
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
