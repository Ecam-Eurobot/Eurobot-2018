#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <BallSeparationLib.h>

#define ID1    (7u)
#define ID2    (1u)
const int CLEAN = 1;
const int DIRTY = 2;
const int BLOCK = 3;

//Dynamixel AX-12A definitions and global variables
//for control of valve used to separate/purify balls/water.
//@param valve_pos is controlled by ROS board.
int valve_pos;
const int servo_speed = 500; //speed for ax-12a movement
const int clean_ball_position = 718;    //still needs to be calibrated
const int dirty_ball_position = 312;
const int block_position = 512;

// Bool variable decides whether to push bee
// or return to initial position. False = initial position
// True = push bee position 
bool pushbee = false;
int pushbee_pos = 180;   // initial/push position for bee still needs to be calibrated
int initialpos = 500;

//Gun declarations. @param dutycycle (0-255) is controlled
//by ROS board.
int dutycycle = 0;
//const int off = 0;
const int i1 = 5;
const int i2 = 4;
const int ena_pwm = 9;     //PWM pin at 490Hz 


//Start ROS handle.
ros::NodeHandle nh;

void moveBee( const std_msgs::Bool & position_msg){
  pushbee = position_msg.data;
  if(pushbee){
    ax12Move(ID2, pushbee_pos, servo_speed);
    }
  else {
    ax12Move(ID2, initialpos, servo_speed);
    }
}

//std_msgs::String pos_msg;
void moveValve(const std_msgs::Int8 & pos_msg){
  valve_pos = pos_msg.data;
  if(valve_pos == CLEAN){
    ax12Move(ID1, clean_ball_position, servo_speed);
    }
  else if (valve_pos == DIRTY){
    ax12Move(ID1, dirty_ball_position, servo_speed);
    }
  else if(valve_pos == BLOCK){
    ax12Move(ID1, block_position, servo_speed);
    }
}

void shootGun( const std_msgs::Int16 & dutycycle_msg){
  dutycycle = dutycycle_msg.data;

  //direction
  digitalWrite(i1, HIGH);
  digitalWrite(i2, LOW); 

  //Drive
  analogWrite(ena_pwm, dutycycle);
  //delay(100);
  //analogWrite(pwm, off);
}

ros::Subscriber<std_msgs::Bool> bee_ctrl("bee_control", &moveBee);
ros::Subscriber<std_msgs::Int8> ballseparator_ctrl("water_purification", &moveValve);
ros::Subscriber<std_msgs::Int16> gun_ctrl("gun_control", &shootGun);

void setup() {
  pinMode(ena_pwm , OUTPUT);
  pinMode(i1, OUTPUT);
  pinMode(i2, OUTPUT);
  nh.initNode();
  nh.subscribe(bee_ctrl);
  nh.subscribe(ballseparator_ctrl);
  nh.subscribe(gun_ctrl);
  ax12Start(servo_speed);
}


//the loop contains is empty.
void loop() {
  nh.spinOnce();
  delay(50);
}
