#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <NewPing.h>

#define TRIGGER_PINL  2   
#define ECHO_PINL    3   

#define TRIGGER_PINR  4   
#define ECHO_PINR     5  

#define TRIGGER_PINB  6   
#define ECHO_PINB     7 

#define TRIGGER_PINF  8   
#define ECHO_PINF     9 

#define MAX_DISTANCE 300 // Maximum distance we want to ping  

NewPing sonarL(TRIGGER_PINL, ECHO_PINL, MAX_DISTANCE);  
NewPing sonarR(TRIGGER_PINR, ECHO_PINR, MAX_DISTANCE);
NewPing sonarB(TRIGGER_PINB, ECHO_PINB, MAX_DISTANCE);  
NewPing sonarF (TRIGGER_PINF, ECHO_PINF, MAX_DISTANCE);

ros::NodeHandle  nh;
 

 
sensor_msgs::Range range_msg;
ros::Publisher pub_range1("/ultrasound1", &range_msg);
ros::Publisher pub_range2("/ultrasound2", &range_msg);
ros::Publisher pub_range3("/ultrasound3", &range_msg);
ros::Publisher pub_range4("/ultrasound4", &range_msg);
 

char frameid[] = "/base_link";

long duration;
 float tmp;

void setup()
{
  nh.initNode();
  nh.advertise(pub_range1);
  nh.advertise(pub_range2);
  nh.advertise(pub_range3);
  nh.advertise(pub_range4);

  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.1;  // fake
  range_msg.min_range = 0.0;
  range_msg.max_range = MAX_DISTANCE;
}

long range_time;

void loop()
{
  //publish the adc value every 50 milliseconds
  //since it takes that long for the sensor to stabilize
  if ( millis() >= range_time ){
    tmp=sonarL.ping_cm();
    range_msg.range = tmp/100;
    range_msg.header.stamp = nh.now();
    pub_range1.publish(&range_msg);

    tmp=sonarR.ping_cm();
    range_msg.range = tmp/100;
    range_msg.header.stamp = nh.now();
    pub_range2.publish(&range_msg);
    
     tmp=sonarB.ping_cm();
    range_msg.range = tmp/100;
    range_msg.header.stamp = nh.now();
    pub_range3.publish(&range_msg);

    tmp=sonarF.ping_cm();
    range_msg.range = tmp/100;
    range_msg.header.stamp = nh.now();
    pub_range4.publish(&range_msg);

    range_time =  millis() + 50;
  }
  nh.spinOnce();
}
