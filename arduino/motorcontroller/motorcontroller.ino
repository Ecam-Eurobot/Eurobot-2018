#include <Wire.h>
#include <PID_v1.h>
#include <ecamlib.h>

#include <Encoder.h>
#include <FlexiTimer2.h>

const char I2C_ADDRESS = 0x08;

// Defines the pins to control the motor driver
// pin 5 conrols the voltage to the motor by a PWM
// pin 4 controls the direction of the motor
const int PWM = 5;
const int DIR = 4;

Encoder motor_encoder = Encoder(2, 3);
const int ENCODER_TICKS_PER_REV = 3200;

// Defines the time between two samples in miliseconds
const int CADENCE_MS = 50;
volatile double dt = CADENCE_MS / 1000.;

// Configuration
bool enabled = false;   // Enable motor output
bool timeout = false;   // Enable timeout to stop the motors if no command for some time

// Motor control variables and PID configuration
float motor_speed = 6.28319;
float gain_p = 15.0;
float gain_i = 5.0;
float gain_d = 0.5;

// Angular velocity
volatile double omega;

// PID 
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint,gain_p,gain_i,gain_d, DIRECT);

// Variable to keep track of the old encoder value
volatile long old_encoder = 0;

void setup() {
    Wire.begin(I2C_ADDRESS);      // join i2c bus
    Wire.onReceive(receiveEvent); // register callback for when we receive data
   
    Serial.begin(9600);

    // set timer 0 divisor to 1 for PWM frequency of 62500.00 Hz
    TCCR0B = TCCR0B & B11111000 | B00000001;

    pinMode(PWM,OUTPUT); 
    pinMode(DIR,OUTPUT);
    
    FlexiTimer2::set(CADENCE_MS, 1/1000., isrt); // Periodic execution of isrt() function
    FlexiTimer2::start();

    myPID.SetMode(AUTOMATIC);
    myPID.SetSampleTime(CADENCE_MS);

    // Motor direction 
    digitalWrite(DIR,LOW);
        
}

void loop() {
    Setpoint = (double) (abs(motor_speed));
    if (motor_speed > 0) {
        digitalWrite(DIR, LOW);
    } else {
        digitalWrite(DIR, HIGH);
    }
    
    Input = abs(omega);
    
    myPID.Compute();
    
    analogWrite(PWM, Output);
    
    Serial.print("Omega : ");
    Serial.println(omega);
    delay(CADENCE_MS);
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
    int deltaEncoder = motor_encoder.read() - old_encoder;
    old_encoder = motor_encoder.read();
    
    // Angular velocity 
    omega = ( (2.0 * 3.141592 * (double)deltaEncoder) / ENCODER_TICKS_PER_REV ) / dt;  // rad/s
}
