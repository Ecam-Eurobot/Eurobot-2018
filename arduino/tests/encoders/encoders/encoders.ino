#include <ros.h>
#include <std_msgs/Int64.h>
#include <ecam_msg/Encoders.h>

ros::NodeHandle  nh;

ecam_msg::Encoders encoders_msg;

ros::Publisher encoders_pub("mecanum/encoder/", &encoders_msg);

void setup()
{
  nh.initNode();
  nh.advertise(encoders_pub);
}

void loop()
{
  encoders_msg.front_left += 20;
  encoders_msg.front_right -= 20;
  encoders_msg.rear_left -= 20;
  encoders_msg.rear_right += 20;
  
  encoders_pub.publish(&encoders_msg);
  
  nh.spinOnce();
  delay(100);
}
