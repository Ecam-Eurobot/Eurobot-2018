#include <PID_v1.h>
#include <ecamlib.h>
#include <Encoder.h>
#include <FlexiTimer2.h>
#include <SPIslave.h>

SpiSlave mySPI;
//SLK  : pin 13
//MISO : pin 12
//MOSI : pin 11 
#define SS 10

// Defines the pins to control the motor driver
// pin 5 conrols the voltage to the motor by a PWM
// pin 4 controls the direction of the motor

#define PWM 5
#define EN 4

Encoder motor_encoder = Encoder(2, 3);
const int ENCODER_TICKS_PER_REV = 3200;

// Defines the time between two samples in miliseconds
const int CADENCE_MS = 50;
volatile double dt = CADENCE_MS / 1000.;


// Motor control variables and PID configuration
union { float f; unsigned char b[4]; } motor_speed ;
union { long l; unsigned char b[4]; }  EncoderState; 

// Angular velocity
volatile double omega = 0;

// PID 
float gain_p= 15.0;
float gain_i = 5.0;
float gain_d = 0.0;  

double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint,gain_p,gain_i,gain_d, DIRECT);

// Variable to keep track of the old encoder value
volatile long old_encoder = 0;

void setup() {
    //init
    motor_speed.f = 6.00;
 
    //Increase PWM SPEED
    TCCR0B = TCCR0B & B11111000 | B00000010;
    pinMode(PWM,OUTPUT); 
    pinMode(EN,OUTPUT);    
    FlexiTimer2::set(CADENCE_MS, 1/1000., isrt); // Periodic execution of isrt() function
    FlexiTimer2::start();
    myPID.SetMode(AUTOMATIC);
    myPID.SetSampleTime(CADENCE_MS);
    // Motor direction 
    digitalWrite(EN,HIGH);
    motor_encoder.write(0);
    mySPI.begin();       
}

void loop() {
    //Reset the data when there isn't any communication.
    if (digitalRead(SS) == HIGH){
      mySPI.reset();
      }
    Setpoint = abs(motor_speed.f);
    Input = abs(omega);
    myPID.Compute();
    if ( motor_speed.f > 0 ){
      analogWrite(PWM, 127 + Output/2);
      }
     else {
      analogWrite(PWM, 127 - Output/2);
      }
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
// Speed measurement 
void isrt(){
    int deltaEncoder = motor_encoder.read() - old_encoder;
    old_encoder = motor_encoder.read();
    EncoderState.l = old_encoder;   
    // Angular velocity 
   omega = ( (2.0 * 3.141592 * (double)deltaEncoder) / ENCODER_TICKS_PER_REV ) / dt;  // rad/s
}


//Interrupt needed by SPI Communication
ISR (SPI_STC_vect) {
    mySPI.com(SPDR); 
    spiReg();//Function called at the end of the communication.   
}

//Function which contains the action binded to each spi registers.
//Register command :
void spiReg(){  
switch (mySPI.command) {
  case 0x10:
      if(mySPI.endTrans) {
          for (int i = 0; i < 4; i++){
              motor_speed.b[i] = mySPI.msg[i];
           }
       }
      break;
   case 0x11:
      SPDR = motor_speed.b[mySPI.dataCount - 2];
      break;
   case 0x50:
      motor_encoder.write(0);
      break;    
   case 0x51:
      SPDR = EncoderState.b[mySPI.dataCount - 2];
      break;    
  } 
}
