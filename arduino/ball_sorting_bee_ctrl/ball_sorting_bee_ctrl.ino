#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Range.h>
#include <NewPing.h>
#include <AX12A.h>

//Ax-12A IDs and baudrate
#define DirectionPin  (10u)
#define BaudRate     (57600ul)
#define ID1          (7u)  //water purification
#define ID2          (1u)  //bee

//Ultrason pin definitions
#define TRIGGER_PINB  2       // us1/back
#define ECHO_PINB     3       // us1/back
#define TRIGGER_PINF  6       // us2/front
#define ECHO_PINF     7       // us2/front
#define TRIGGER_PINL  8       // us3/left
#define ECHO_PINL     10      // us3/left
#define TRIGGER_PINR  11      // us4/right
#define ECHO_PINR     12      // us4/right
#define MAX_DISTANCE  300     // Maximum distance we want to ping  

//Gun pin definitions
#define i1            5
#define i2            4
#define ena_pwm       9     //PWM pin at 490Hz 

//Water purification definitions
#define clean_ball_position 90
#define dirty_ball_position 916

//water purification declarations
int servo_speed = 500; //speed for ax-12a movement

//Dynamixel AX-12A definitions and global variables
//for control of valve used to separate/purify balls/water.
//@param valve_pos is controlled by ROS board.
int initial_pos_purifier = 512;
int initial_pos_bee = 552;
int valve_pos;

//Ultrasound declarations
char frameid[] = "/base_link";
long duration;
float tmp;
long range_time;


NewPing sonarB(TRIGGER_PINB, ECHO_PINB, MAX_DISTANCE);  // back us
NewPing sonarF(TRIGGER_PINF, ECHO_PINF, MAX_DISTANCE);  // front us
NewPing sonarL(TRIGGER_PINL, ECHO_PINL, MAX_DISTANCE);  // left us
NewPing sonarR(TRIGGER_PINR, ECHO_PINR, MAX_DISTANCE); // right us

//Start ROS handle.
ros::NodeHandle nh;

void moveBee( const std_msgs::Bool & position_msg){
  
  // Bool variable decides whether to push bee
  // or return to initial position. False = initial position
  // True = push bee position 
  bool pushbee = false;
  int pushbee_pos = 30;   // initial/push position for bee still needs to be calibrated
  
  pushbee = position_msg.data;
  if(pushbee){
    //ax12Move(ID2, pushbee_pos, servo_speed);
    ax12a.moveSpeed(ID2, pushbee_pos, servo_speed);
    }
  else {
    //ax12Move(ID2, initialpos, servo_speed);
    ax12a.moveSpeed(ID2, initial_pos_bee, servo_speed);
    }
}

void moveValve(const std_msgs::Int16 & pos_msg){
  
  valve_pos = pos_msg.data;
  if(valve_pos == 1){
    ax12a.moveSpeed(ID1, clean_ball_position, servo_speed);
    // "Shake"/rotate purifier between goal position and
    // 10 degrees to assure all balls enter
//    while(counter <= 4){
//      clean_ball_position = clean_ball_position + 20;
//      delay(1000);
//      ax12a.moveSpeed(ID1, clean_ball_position, servo_speed);
//      clean_ball_position = clean_ball_position - 20;
//      delay(1000);
//      ax12a.moveSpeed(ID1, clean_ball_position, servo_speed);
//      counter++;      
//    }
    }
  else if (valve_pos == 2){
    ax12a.moveSpeed(ID1, dirty_ball_position, servo_speed);
    }
  else if(valve_pos == 3){
    ax12a.moveSpeed(ID1, initial_pos_purifier, servo_speed);
    }
}

void shootGun( const std_msgs::Int16 & dutycycle_msg){
   //Gun declarations. @param dutycycle (0-255) is controlled
  //by ROS board.
  int dutycycle = 0;
  dutycycle = dutycycle_msg.data;

  //direction
  digitalWrite(i1, LOW);
  digitalWrite(i2, HIGH); 

  //Drive
  analogWrite(ena_pwm, dutycycle);
}

// Function that pings ultrasound sensors.
// Publish the adc value every 50 milliseconds
// since it takes that long for the sensor to stabilize
void checkSensors(void);
void shake(void);

ros::Subscriber<std_msgs::Bool> bee_ctrl("bee_control", &moveBee);
ros::Subscriber<std_msgs::Int16> ballseparator_ctrl("water_purification", &moveValve);
ros::Subscriber<std_msgs::Int16> gun_ctrl("gun_control", &shootGun);

sensor_msgs::Range range_msg_rear;
sensor_msgs::Range range_msg_front;
sensor_msgs::Range range_msg_left;
sensor_msgs::Range range_msg_right;
ros::Publisher pub_range1("ultrasound_rear", &range_msg_rear);
ros::Publisher pub_range2("ultrasound_front", &range_msg_front);
ros::Publisher pub_range3("ultrasound_left", &range_msg_left);
ros::Publisher pub_range4("ultrasound_right", &range_msg_right);


void setup() {
    
  pinMode(ena_pwm , OUTPUT);
  pinMode(i1, OUTPUT);
  pinMode(i2, OUTPUT);
  
  //Start-up
  ax12a.begin(BaudRate, DirectionPin, &Serial);
  //Remove endless rotation
  ax12a.setEndless(ID1, OFF);
  ax12a.setEndless(ID2, OFF);
  //move into initial position
  ax12a.moveSpeed(ID1, initial_pos_purifier, servo_speed);
  ax12a.moveSpeed(ID2, initial_pos_bee, servo_speed);
//  delay(3000);
//  ax12a.moveSpeed(ID2, 30, servo_speed);  //Calibrated push bee pos old /new position needs to be calibrated
//  delay(3000);
//  ax12a.moveSpeed(ID1, 916, servo_speed);  //Calibrated dirty pos
//  delay(3000);
//  ax12a.moveSpeed(ID1, 90, servo_speed);  //Calibrated clean pos
  
  nh.initNode();
  
  nh.subscribe(bee_ctrl);
  nh.subscribe(ballseparator_ctrl);
  nh.subscribe(gun_ctrl);
  
  nh.advertise(pub_range1);
  nh.advertise(pub_range2);
  nh.advertise(pub_range3);
  nh.advertise(pub_range4);

  range_msg_rear.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_rear.header.frame_id =  "ultrasound_rear";
  range_msg_rear.field_of_view = 0.3665;  // fake
  range_msg_rear.min_range = 0.0;
  range_msg_rear.max_range = MAX_DISTANCE;
  
  range_msg_front.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_front.header.frame_id =  "ultrasound_front";
  range_msg_front.field_of_view = 0.3665;  // fake
  range_msg_front.min_range = 0.0;
  range_msg_front.max_range = MAX_DISTANCE; 
   
  range_msg_left.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_left.header.frame_id =  "ultrasound_left";
  range_msg_left.field_of_view = 0.3665;  // fake
  range_msg_left.min_range = 0.0;
  range_msg_left.max_range = MAX_DISTANCE;  
  
  range_msg_right.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_right.header.frame_id =  "ultrasound_right";
  range_msg_right.field_of_view = 0.3665;  // fake
  range_msg_right.min_range = 0.0;
  range_msg_right.max_range = MAX_DISTANCE;
}

//the loop contains is empty.
void loop() {
  checkSensors();
  nh.spinOnce();
  //delay(50);
  shake();    
}


// Function that pings ultrasound sensors.
// Publish the adc value every 50 milliseconds
// since it takes that long for the sensor to stabilize
void checkSensors(void)
{
  if ( millis() >= range_time ){
    tmp=sonarL.ping_cm();
    range_msg_rear.range = tmp/100;
    range_msg_rear.header.stamp = nh.now();
    pub_range1.publish(&range_msg_rear);

    tmp=sonarR.ping_cm();
    range_msg_front.range = tmp/100;
    range_msg_front.header.stamp = nh.now();
    pub_range2.publish(&range_msg_front);
    
     tmp=sonarB.ping_cm();
    range_msg_left.range = tmp/100;
    range_msg_left.header.stamp = nh.now();
    pub_range3.publish(&range_msg_left);

    tmp=sonarF.ping_cm();
    range_msg_right.range = tmp/100;
    range_msg_right.header.stamp = nh.now();
    pub_range4.publish(&range_msg_right);

    range_time =  millis() + 50;
  }
}

//Move purifier to and fro in order to make
//sure all balls enter compartment
void shake(void)
{
    int dir = 0;
    
    if (valve_pos == 1){
    if (dir == 0){
      if (ax12a.readPosition(ID1) != clean_ball_position + 200){
        ax12a.moveSpeed(ID1, clean_ball_position + 200, servo_speed);
      }
      else {
        dir = 1;
      }
      
    }
    else if (dir == 1){
      if (ax12a.readPosition(ID1) != clean_ball_position){
        ax12a.moveSpeed(ID1, clean_ball_position - 200, servo_speed);
      }
      else {
        dir = 0;
      }
    }

  }
  if (valve_pos == 2){
    if (dir == 0){
      if (ax12a.readPosition(ID1) != dirty_ball_position + 200){
        ax12a.moveSpeed(ID1, dirty_ball_position - 200, servo_speed);
      }
      else {
        dir = 1;
      }
      
    }
    else if (dir == 1){
      if (ax12a.readPosition(ID1) != dirty_ball_position){
        ax12a.moveSpeed(ID1, dirty_ball_position + 200, servo_speed);
      }
      else {
        dir = 0;
      }
    }

  }
}
