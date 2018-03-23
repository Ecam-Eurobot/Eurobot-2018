//SPI Manager drive by ROSSERIAL.
#include <SPIManager.h>
#include <SPI.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Empty.h>
#include <ecam_msg/Encoders.h>
ros::NodeHandle nh;
std_msgs::Float32 speed_FL;
std_msgs::Empty empty_msg;
ecam_msg::Encoders encoders_msg;






#define SERIAL_BAUD 57600


#define FL_SS  9
SPIManager connFL(FL_SS);
#define FR_SS  2
SPIManager connFR(FR_SS);
#define BL_SS  10
SPIManager connBL(BL_SS);
#define BR_SS  3
SPIManager connBR(BR_SS);


union{ float f; unsigned char b[4];} front_left_speed;
void message_to_FL( const std_msgs::Float32 & toggle_msg){
  front_left_speed.f = toggle_msg.data;
}

union{ float f; unsigned char b[4];} front_right_speed;
void message_to_FR( const std_msgs::Float32 & toggle_msg){
  front_right_speed.f  = toggle_msg.data;
}

union{ float f; unsigned char b[4];} rear_left_speed;
void message_to_BL( const std_msgs::Float32 & toggle_msg){
  rear_left_speed.f = toggle_msg.data;
}

union{ float f; unsigned char b[4];} rear_right_speed;
void message_to_BR( const std_msgs::Float32 & toggle_msg){
  rear_right_speed.f = toggle_msg.data;
}

//float noData;
//void encoder_reset(const std_msgs::Empty& toggle_msg){
//  connFL.initialize();
//  connFL.readLongData(0x51,0x04,front_left_speed.b);
//  connFR.initialize();
//  connFR.readLongData(0x50,0x04,front_right_speed.b);
//  connBL.initialize();
//  connBL.readLongData(0x50,0x04,rear_left_speed.b);
//  connBR.initialize();
//  connBR.readLongData(0x50,0x04,rear_right_speed.b);
//}
//


void poll_encoder(){
  connFL.initialize();
  long frontleft = connFL.readLongData(0x51,0x04,front_left_speed.b);
  connFL.writeData(0x10, 0x04, front_left_speed.b);
  connFR.initialize();
  long frontright = connFR.readLongData(0x51,0x04,front_right_speed.b);
  connFR.writeData(0x10, 0x04, front_right_speed.b );
  connBL.initialize();
  long rearleft = connBL.readLongData(0x51,0x04,rear_left_speed.b);
  connBL.writeData(0x10, 0x04, rear_left_speed.b); 
  connBR.initialize();
  long rearright = connBR.readLongData(0x51,0x04,rear_right_speed.b);
  connBR.writeData(0x10, 0x04, rear_right_speed.b);
  
  encoders_msg.front_left = frontleft;
  encoders_msg.front_right = frontright;
  encoders_msg.rear_left = rearleft;
  encoders_msg.rear_right = rearright;
  }
  
ros::Subscriber<std_msgs::Float32> mecanumFL("/motor/front/left" , &message_to_FL);
ros::Subscriber<std_msgs::Float32> mecanumFR("/motor/front/right", &message_to_FR);
ros::Subscriber<std_msgs::Float32> mecanumBL("/motor/rear/left"  , &message_to_BL);
ros::Subscriber<std_msgs::Float32> mecanumBR("/motor/rear/right" , &message_to_BR);
//ros::Subscriber<std_msgs::Empty> encoder_reset_sub("mecanum/encoder/reset" , &encoder_reset);
ros::Publisher encoders_pub("mecanum/encoder", &encoders_msg);
ros::Publisher speedFL("/motor/front/left/speed", &speed_FL);



void loopstart(){
  front_left_speed.f = 0.00;
  front_right_speed.f = 0.00;
  rear_left_speed.f = 0.00;
  rear_right_speed.f = 0.00;
  }

void setup() {
  nh.initNode();
  nh.advertise(encoders_pub);
  nh.advertise(speedFL);
//  nh.subscribe(encoder_reset_sub);
  nh.subscribe(mecanumFL);
  nh.subscribe(mecanumFR);
  nh.subscribe(mecanumBL);
  nh.subscribe(mecanumBR);
  connFL.initialize();
  connFR.initialize();
  connBL.initialize();
  connBR.initialize();
  loopstart();
  Serial.begin(SERIAL_BAUD);
}


//the loop contains is empty.
void loop() {
  poll_encoder();
  encoders_pub.publish(&encoders_msg);
  nh.spinOnce();
  delay(10);
}






