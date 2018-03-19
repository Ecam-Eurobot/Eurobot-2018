//SPI Manager drive by ROSSERIAL.
#include <SPIManager.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Empty.h>
#include <ecam_msg/Encoders.h>
#include <SPI.h>

#define SERIAL_BAUD 57600
#define FL_SS  9
#define FR_SS  2
#define BL_SS  10
#define BR_SS  3

SPIManager connFL(FL_SS);
SPIManager connFR(FR_SS);
SPIManager connBL(BL_SS);
SPIManager connBR(BR_SS);

union cvt {
float val;
unsigned char b[4];
} x;

//Start ROS handle.
ros::NodeHandle nh;

std_msgs::Float32 str_msg;
std_msgs::Empty empty_msg;
ecam_msg::Encoders encoders_msg;

void message_to_FL( const std_msgs::Float32 & toggle_msg){
  x.val = toggle_msg.data;
  connFL.initialize();
  connFL.writeData(0x10, 0x04, x.b);
}

void message_to_FR( const std_msgs::Float32 & toggle_msg){
  x.val = toggle_msg.data;
  connFR.initialize();
  connFR.writeData(0x10, 0x04, x.b);
}

void message_to_BL( const std_msgs::Float32 & toggle_msg){
  x.val =toggle_msg.data;
  connBL.initialize();
  connBL.writeData(0x10, 0x04, x.b);
}

void message_to_BR( const std_msgs::Float32 & toggle_msg){
  x.val = toggle_msg.data;
  connBR.initialize();
  connBR.writeData(0x10, 0x04, x.b);
}

float noData;
void encoder_reset(const std_msgs::Empty& toggle_msg){
  connFL.initialize();
  noData = connFL.readData(0x50);
  connFL.end();
  connFR.initialize();
  noData  = connFR.readData(0x50);
  connFR.end();
  connBL.initialize();
  noData  = connBR.readData(0x50);
  connBL.end();
  connBR.initialize();
  noData  = connBL.readData(0x50);
  connBR.end();
}


void poll_encoder(){
  connFL.initialize();
  encoders_msg.front_left = connFL.readLongData(0x51);
  connFL.end();
  connFR.initialize();
  encoders_msg.front_right = connFR.readLongData(0x51);
  connFR.end();
  connBL.initialize();
  encoders_msg.rear_right = connBR.readLongData(0x51);
  connBL.end();
  connBR.initialize();
  encoders_msg.rear_left = connBL.readLongData(0x51);
  connBR.end();
  }

ros::Subscriber<std_msgs::Float32> mecanumFL("/motor/front/left" , &message_to_FL);
ros::Subscriber<std_msgs::Float32> mecanumFR("/motor/front/right", &message_to_FR);
ros::Subscriber<std_msgs::Float32> mecanumBL("/motor/rear/left"  , &message_to_BL);
ros::Subscriber<std_msgs::Float32> mecanumBR("/motor/rear/right" , &message_to_BR);
ros::Subscriber<std_msgs::Empty> encoder_reset_sub("mecanum/encoder/reset" , &encoder_reset);
ros::Publisher encoders_pub("mecanum/encoder", &encoders_msg);

void setup() {
  nh.initNode();
  nh.advertise(encoders_pub);
  nh.subscribe(encoder_reset_sub);
  nh.subscribe(mecanumFL);
  nh.subscribe(mecanumFR);
  nh.subscribe(mecanumBL);
  nh.subscribe(mecanumBR);
  connFL.initialize();
  connFR.initialize();
  connBL.initialize();
  connBR.initialize();
  Serial.begin(SERIAL_BAUD);
}


//the loop contains is empty.
void loop() {
  encoders_pub.publish(&encoders_msg);
  poll_encoder();
  nh.spinOnce();
  delay(20);
}



