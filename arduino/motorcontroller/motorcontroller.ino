#include <Wire.h>
#include <PID_v1.h>
#include <ecamlib.h>

#include <FlexiTimer2.h>
#include <digitalWriteFast.h> 

const char I2C_ADDRESS = 0x08;

// Configuration
bool enabled = false;   // Enable motor output
bool timeout = false;   // Enable timeout to stop the motors if no command for some time

// Motor control variables and PID configuration
float motor_speed = 0.0;
float gain_p = 1.0;
float gain_i = 2.0;
float gain_d = 0.5;

volatile double omega;

double Setpoint, Input, Output;
// Parameter working but slow myPID(&Input, &Output, &Setpoint,1,2,0.5, DIRECT);
PID myPID(&Input, &Output, &Setpoint,gain_p,gain_i,gain_d, DIRECT);

// Incremental encoder pins
int interrupPinEncoderA = 3; // Corresponding to the interrupt pin 0 for Arduino
int interrupPinEncoderB = 2; // Corresponding to the interrupt pin 1 for Arduino 

volatile long ticksCodeur = 0;

// Motor configuration 
int PWM1 = 5; // PWM control of motor 1 - Arduino Pin 5
int DIR1 = 4; // Direction of the motor LOW or HIGH - Arduino Pin 2

// Sampling cadence (ms)
#define CADENCE_MS 50
volatile double dt = CADENCE_MS/1000.;
volatile double temps = -CADENCE_MS/1000.;

void setup() {
    Wire.begin(I2C_ADDRESS);      // join i2c bus
    Wire.onReceive(receiveEvent); // register callback for when we receive data
    Serial.begin(9600);
    Serial.flush();

    pinMode(PWM1,OUTPUT); 
    pinMode(DIR1,OUTPUT);

    pinMode(interrupPinEncoderA, INPUT);      // Encoder1 Pin A (Digital) 
    pinMode(interrupPinEncoderB, INPUT);      // Encoder1 Pin B (Digital)
    attachInterrupt(1, GestionInterruptionCodeurPinA, CHANGE); // Interruption routine configuration
    attachInterrupt(0, GestionInterruptionCodeurPinB, CHANGE); // Interruption routine configuration

    ticksCodeur = 0; // Encoder pulse counter
    
    FlexiTimer2::set(CADENCE_MS, 1/1000., isrt); // Periodic execution of isrt() function
    FlexiTimer2::start();

    myPID.SetMode(AUTOMATIC);
    myPID.SetSampleTime(CADENCE_MS);

    // Motor direction 
    digitalWrite(DIR1,LOW);
        
}

void loop() {
    
    Setpoint = (double)(abs(motor_speed));
    if (motor_speed > 0) {digitalWrite(DIR1,LOW);}
    else {digitalWrite(DIR1,HIGH);}
    
    Input = abs(omega);
    
    myPID.Compute();
    
    analogWrite(PWM1, Output);
    
    Serial.print("Omega : ");
    Serial.println(omega);
    delay(1);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
    if (Wire.available() >= 2) {
        unsigned char reg = Wire.read();

        switch (reg) {
            // Configuration
            case 0x00:
                Serial.println("Tried to change the configuration, but it's not handled yet...");
                break;

            // Set motor speed
            case 0x10:
                // Expecting a 4 byte float value representing
                // angular velocity
                if (Wire.available() >= 4) {
                    unsigned char b0 = Wire.read();
                    unsigned char b1 = Wire.read();
                    unsigned char b2 = Wire.read();
                    unsigned char b3 = Wire.read();
                    motor_speed = bytesToFloat(b0, b1, b2, b3);
                    Serial.print("Motor speed set to ");
                    Serial.print(motor_speed, 3);
                    Serial.println(" rad/s");
                }
                break;

            // Set PID gains
            case 0x20:
                // Expecting a 4 byte float value representing
                // proportional gain for the PID
                if (Wire.available() >= 4) {
                    unsigned char b0 = Wire.read();
                    unsigned char b1 = Wire.read();
                    unsigned char b2 = Wire.read();
                    unsigned char b3 = Wire.read();
                    gain_p = bytesToFloat(b0, b1, b2, b3);
                    Serial.print("Proportional gain set to ");
                    Serial.println(gain_p, 3);
                }
                break;
            case 0x21:
                // Expecting a 4 byte float value representing
                // integral gain for the PID
                if (Wire.available() >= 4) {
                    unsigned char b0 = Wire.read();
                    unsigned char b1 = Wire.read();
                    unsigned char b2 = Wire.read();
                    unsigned char b3 = Wire.read();
                    gain_i = bytesToFloat(b0, b1, b2, b3);
                    Serial.print("Integral gain set to ");
                    Serial.println(gain_i, 3);
                }
                break;
            case 0x22:
                // Expecting a 4 byte float value representing
                // derivative gain for the PID
                if (Wire.available() >= 4) {
                    unsigned char b0 = Wire.read();
                    unsigned char b1 = Wire.read();
                    unsigned char b2 = Wire.read();
                    unsigned char b3 = Wire.read();
                    gain_d = bytesToFloat(b0, b1, b2, b3);
                    Serial.print("Derivative gain set to ");
                    Serial.println(gain_d, 3);
                }
                break;
            
            default:
                Serial.println("Unexcpected register access over I2C");
                break;
        }
    }
}

// Speed measurement 
void isrt(){
  int codeurDeltaPos;
 
   // Number of ticks since last time
  codeurDeltaPos = ticksCodeur;
  ticksCodeur = 0;
 
  // Speed calculation 
  omega = ((2.*3.141592*((double)codeurDeltaPos))/1920)/dt;  // rad/s

  temps += dt;
}


// Interruption routine attached to the encoder channel A
void GestionInterruptionCodeurPinA(){
  if (digitalReadFast2(interrupPinEncoderA) == digitalReadFast2(interrupPinEncoderB)) {ticksCodeur--;}
  else {ticksCodeur++;}
}

// Interruption routine attached to the encoder channel A
void GestionInterruptionCodeurPinB(){
  if (digitalReadFast2(interrupPinEncoderA) == digitalReadFast2(interrupPinEncoderB)) {ticksCodeur++;}
  else {ticksCodeur--;}
}
