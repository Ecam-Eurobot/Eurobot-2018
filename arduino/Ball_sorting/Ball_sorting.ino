#include <Wire.h>
#include <BallSeparationLib.h>
#include <ecamlib.h>

const char I2C_ADDRESS = 0x60;

//Servo definitions
int cmd_ServoPos = 0; // position requested by ROS

//Electromagnet global definitions


//Gun global definitions

void setup() {
  Serial.begin(9600);
  servoStart();

  Wire.begin(I2C_ADDRESS);      // join i2c bus
  Wire.onReceive(receiveEvent); // register callback for when we receive data
}

void loop() {
  delay(100);
}

void receiveEvent(int numBytes) {
  const char* teamColour = "";

  if (Wire.available() > 2) {
     unsigned char reg = Wire.read();

     switch (reg) {
            // Configuration. Detect team color and send answer
            // to master (ROS board)
            case 0x00:
                Serial.println("Initialising color sensor");
                if(csStartUpRoutine()){
                  Serial.println("Found sensor");
                  teamColour = csCheckTeamColor();
                  Serial.println(teamColour); 
                  //Send I2c message of success and team colour to ROS               
                }else{
                  Serial.println("No TCS34725 found ... check your connections");
                  //Send I2c message of failure to ROS
                }               
                break;

            // Set ball valve in position 1 or position 2 for clean/dirty 
            // balls collection
            case 0x01:
                //Expecting 2 bytes (int) of data for position info
                if (Wire.available() >= 2) {
                  unsigned char b0 = Wire.read();
                  unsigned char b1 = Wire.read();
                  cmd_ServoPos = bytesToInt(b0,b1);
                  servoComputePos(cmd_ServoPos);
                }
                break;

            // Launch stepper motor 
            case 0x02:
                if (Wire.available() >= 4) {
                }
                break;

            // Release electromagnet for dirty balls trap    
            case 0x03:
                if (Wire.available() >= 4) {
                }
                break;

            // Initialise and set up Gun    
            case 0x04:
                if (Wire.available() >= 4) {
                }
                break;

            // Start shooting routine    
            case 0x05:
                if (Wire.available() >= 4) {
                }
                break;
            
            default:
                Serial.println("Unexcpected register access over I2C");
                break;
        }
    }
  
  }
