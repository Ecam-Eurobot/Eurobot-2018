#include <PID_v1.h>
#include <ecamlib.h>
#include <Encoder.h>
#include <FlexiTimer2.h>

// ROS dependencies
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Empty.h>
#include <ecam_msg/Encoders.h>

// Wiring 
// FL motor
// HBridge ENA : 3; IN1 : 28; IN2 : 26 
// Motor YELLOW : 42; WHITE : 20

// FR motor
// HBridge ENA : 5; IN3 : 22; IN4 : 24 
// Motor YELLOW : 40; WHITE : 2

// BL motor
// HBridge ENA : 11; IN3 : 29; IN4 : 27 
// Motor YELLOW : 38; WHITE : 21

// BR motor
// HBridge ENA : 6; IN1 : 25; IN2 : 23 
// Motor YELLOW : 36; WHITE : 19


// The use of flexitimer2 (using TIMER2) renders the 
// PWM signal of pins 9 and 10 on the arduino MEGA unusable

// Front Left motor pins 
#define PWM_FL 3
#define IN1_FL 28
#define IN2_FL 26

// Front Right motor pins
#define PWM_FR 5
#define IN1_FR 22
#define IN2_FR 24

// Back Left motor pins
#define PWM_BL 11
#define IN1_BL 27
#define IN2_BL 29

// Back Right motor pins
#define PWM_BR 6
#define IN1_BR 23
#define IN2_BR 25 

Encoder motor_encoder_FL = Encoder(20, 42);
Encoder motor_encoder_FR = Encoder(2, 40);
Encoder motor_encoder_BL = Encoder(21, 38);
Encoder motor_encoder_BR = Encoder(19, 36);

const int ENCODER_TICKS_PER_REV = 3200;
const int CADENCE_MS = 10;
volatile double dt = CADENCE_MS / 1000.;

// Angular velocity
volatile double angular_speed_FL = 0;
volatile double angular_speed_FR = 0;
volatile double angular_speed_BL = 0;
volatile double angular_speed_BR = 0;

// PID 
float gain_p = 80.0;
float gain_i = 5.0;
float gain_d = 0.0;  

double setpoint_FL, input_FL, output_FL;
PID PID_FL(&input_FL, &output_FL, &setpoint_FL,gain_p,gain_i,gain_d, DIRECT);
double setpoint_FR, input_FR, output_FR;
PID PID_FR(&input_FR, &output_FR, &setpoint_FR,gain_p,gain_i,gain_d, DIRECT);
double setpoint_BL, input_BL, output_BL;
PID PID_BL(&input_BL, &output_BL, &setpoint_BL,gain_p,gain_i,gain_d, DIRECT);
double setpoint_BR, input_BR, output_BR;
PID PID_BR(&input_BR, &output_BR, &setpoint_BR,gain_p,gain_i,gain_d, DIRECT);

// Variable to keep track of the old encoder value
volatile long old_encoder_FL = 0;
volatile long old_encoder_FR = 0;
volatile long old_encoder_BL = 0;
volatile long old_encoder_BR = 0;

float velocity_FL = 0;
float velocity_FR = 0;
float velocity_BL = 0;
float velocity_BR = 0;

bool flag_encoders = false;


// ROS
ros::NodeHandle nh;

std_msgs::Float32 ros_velocity_FL;
std_msgs::Float32 ros_velocity_FR;
std_msgs::Float32 ros_velocity_BL;
std_msgs::Float32 ros_velocity_BR;

std_msgs::Empty ros_empty_msg;

const long SERIAL_BAUDRATE = 115200;

// ROS Subscribers
void set_FL_velocity( const std_msgs::Float32 & velocity_msg){
    velocity_FL = velocity_msg.data;
}

void set_FR_velocity( const std_msgs::Float32 & velocity_msg){
    velocity_FR = velocity_msg.data;
}

void set_BL_velocity( const std_msgs::Float32 & velocity_msg){
    velocity_BL = velocity_msg.data;
}

void set_BR_velocity( const std_msgs::Float32 & velocity_msg){
    velocity_BR = velocity_msg.data;
}

ros::Subscriber<std_msgs::Float32> mecanumFL("/motor/front_left/velocity" , &set_FL_velocity);
ros::Subscriber<std_msgs::Float32> mecanumFR("/motor/front_right/velocity", &set_FR_velocity);
ros::Subscriber<std_msgs::Float32> mecanumBL("/motor/back_left/velocity"  , &set_BL_velocity);
ros::Subscriber<std_msgs::Float32> mecanumBR("/motor/back_right/velocity" , &set_BR_velocity);

// ROS Publishers
std_msgs::Float32 ros_encoder_FL;
std_msgs::Float32 ros_encoder_FR;
std_msgs::Float32 ros_encoder_BL;
std_msgs::Float32 ros_encoder_BR;

ros::Publisher encoder_pub_FL("motor/front_left/encoder", &ros_encoder_FL);
ros::Publisher encoder_pub_FR("motor/front_right/encoder", &ros_encoder_FR);
ros::Publisher encoder_pub_BL("motor/back_left/encoder", &ros_encoder_BL);
ros::Publisher encoder_pub_BR("motor/back_right/encoder", &ros_encoder_BR);

void setup() {

    pinMode(PWM_FL,OUTPUT);
    pinMode(IN1_FL,OUTPUT); 
    pinMode(IN2_FL,OUTPUT); 
    pinMode(PWM_FR,OUTPUT);
    pinMode(IN1_FR,OUTPUT); 
    pinMode(IN2_FR,OUTPUT);  
    pinMode(PWM_BL,OUTPUT);
    pinMode(IN1_BL,OUTPUT); 
    pinMode(IN2_BL,OUTPUT);
    pinMode(PWM_BR,OUTPUT);
    pinMode(IN1_BR,OUTPUT); 
    pinMode(IN2_BR,OUTPUT);  

    FlexiTimer2::set(CADENCE_MS, compute_velocity); // Periodic execution of isrt() function
    FlexiTimer2::start();

    PID_FL.SetMode(AUTOMATIC);
    PID_FL.SetSampleTime(CADENCE_MS);
    PID_FL.SetOutputLimits(-255,255);
    PID_FR.SetMode(AUTOMATIC);
    PID_FR.SetSampleTime(CADENCE_MS);
    PID_FR.SetOutputLimits(-255,255);
    PID_BR.SetMode(AUTOMATIC);
    PID_BR.SetSampleTime(CADENCE_MS);
    PID_BR.SetOutputLimits(-255,255);
    PID_BL.SetMode(AUTOMATIC);
    PID_BL.SetSampleTime(CADENCE_MS);   
    PID_BL.SetOutputLimits(-255,255);  

    // ROS
    nh.initNode();

    nh.subscribe(mecanumFL);
    nh.subscribe(mecanumFR);
    nh.subscribe(mecanumBL);
    nh.subscribe(mecanumBR);

    nh.advertise(encoder_pub_FL);
    nh.advertise(encoder_pub_FR);
    nh.advertise(encoder_pub_BL);
    nh.advertise(encoder_pub_BR);

    // float gain_p;
    // float gain_i;
    // float gain_d;
    // if (!nh.getParam("~motor/front_left/pid/gain_p", &gain_p)) { gain_p = 80; }
    // if (!nh.getParam("~motor/front_left/pid/gain_i", &gain_i)) { gain_i = 5; }
    // if (!nh.getParam("~motor/front_left/pid/gain_d", &gain_d)) { gain_d = 0; }
    // PID_FL.SetTunings(gain_p, gain_i, gain_d);

    // if (!nh.getParam("/motor/front_right/pid/gain_p", &gain_p)) { gain_p = 80; }
    // if (!nh.getParam("/motor/front_right/pid/gain_i", &gain_i)) { gain_i = 5; }
    // if (!nh.getParam("/motor/front_right/pid/gain_d", &gain_d)) { gain_d = 0; }
    // PID_FR.SetTunings(gain_p, gain_i, gain_d);

    // if (!nh.getParam("/motor/back_left/pid/gain_p", &gain_p)) { gain_p = 80; }
    // if (!nh.getParam("/motor/back_left/pid/gain_i", &gain_i)) { gain_i = 5; }
    // if (!nh.getParam("/motor/back_left/pid/gain_d", &gain_d)) { gain_d = 0; }
    // PID_BL.SetTunings(gain_p, gain_i, gain_d);

    // if (!nh.getParam("/motor/back_right/pid/gain_p", &gain_p)) { gain_p = 80; }
    // if (!nh.getParam("/motor/back_right/pid/gain_i", &gain_i)) { gain_i = 5; }
    // if (!nh.getParam("/motor/back_right/pid/gain_d", &gain_d)) { gain_d = 0; }
    // PID_BR.SetTunings(gain_p, gain_i, gain_d);
    

    Serial.begin(SERIAL_BAUDRATE);
}

void loop() {
  
    setpoint_FL = velocity_FL;
    setpoint_FR = velocity_FR;
    setpoint_BR = velocity_BR;
    setpoint_BL = velocity_BL;
    
    input_FL = angular_speed_FL;
    input_FR = angular_speed_FR;
    input_BL = angular_speed_BL;
    input_BR = angular_speed_BR;
    
    PID_FL.Compute();
    PID_FR.Compute();
    PID_BL.Compute();
    PID_BR.Compute();

    // Handle direction with H bridge pin writing 
    if (output_FL < 0 ){
      digitalWrite(IN1_FL, HIGH);
      digitalWrite(IN2_FL, LOW);
    }
    else {
      digitalWrite(IN1_FL, LOW);
      digitalWrite(IN2_FL, HIGH);
    }

    if (output_FR < 0 ){
      digitalWrite(IN1_FR, HIGH);
      digitalWrite(IN2_FR, LOW);
    }
    else {
      digitalWrite(IN1_FR, LOW);
      digitalWrite(IN2_FR, HIGH);
    }

    if (output_BL < 0 ){
      digitalWrite(IN1_BL, HIGH);
      digitalWrite(IN2_BL, LOW);
    }
    else {
      digitalWrite(IN1_BL, LOW);
      digitalWrite(IN2_BL, HIGH);
    }

    if (output_BR <  0 ){
      digitalWrite(IN1_BR, HIGH);
      digitalWrite(IN2_BR, LOW);
    }
    else {
      digitalWrite(IN1_BR, LOW);
      digitalWrite(IN2_BR, HIGH);
    }

    analogWrite(PWM_FL, abs(output_FL));
    analogWrite(PWM_FR, abs(output_FR));
    analogWrite(PWM_BL, abs(output_BL));
    analogWrite(PWM_BR, abs(output_BR));

    if (flag_encoders) {
        ros_encoder_FL.data = old_encoder_FL;
        encoder_pub_FL.publish(&ros_encoder_FL);

        ros_encoder_FR.data = old_encoder_FR;
        encoder_pub_FR.publish(&ros_encoder_FR);

        ros_encoder_BL.data = old_encoder_BL;
        encoder_pub_BL.publish(&ros_encoder_BL);

        ros_encoder_BR.data = old_encoder_BR;
        encoder_pub_BR.publish(&ros_encoder_BR);

        flag_encoders = false;
    }

    nh.spinOnce(); 
}

// this function is registered as an event, see setup()
// Speed measurement 
void compute_velocity(){
    // FL motor
    int delta_encoder_FL = motor_encoder_FL.read() - old_encoder_FL;
    old_encoder_FL = motor_encoder_FL.read();
    angular_speed_FL = ( (2.0 * 3.141592 * (double)delta_encoder_FL) / ENCODER_TICKS_PER_REV ) / dt;  // rad/s

    // FR motor
    int delta_encoder_FR = motor_encoder_FR.read() - old_encoder_FR;
    old_encoder_FR = motor_encoder_FR.read();
    angular_speed_FR = -( (2.0 * 3.141592 * (double)delta_encoder_FR) / ENCODER_TICKS_PER_REV ) / dt;  // rad/s

    // BL motor
    int delta_encoder_BL = motor_encoder_BL.read() - old_encoder_BL;
    old_encoder_BL = motor_encoder_BL.read();
    angular_speed_BL = ( (2.0 * 3.141592 * (double)delta_encoder_BL) / ENCODER_TICKS_PER_REV ) / dt;  // rad/s

    // BR motor
    int delta_encoder_BR = motor_encoder_BR.read() - old_encoder_BR;
    old_encoder_BR = motor_encoder_BR.read();
    angular_speed_BR = -( (2.0 * 3.141592 * (double)delta_encoder_BR) / ENCODER_TICKS_PER_REV ) / dt;  // rad/s

    flag_encoders = true;

}
