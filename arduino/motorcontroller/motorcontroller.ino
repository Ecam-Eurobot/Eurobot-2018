#include <Wire.h>
#include <Encoder.h>
#include <PID_v1.h>

#include <ecamlib.h>

const char I2C_ADDRESS = 0x08;

// Configuration
bool enabled = false;   // Enable motor output
bool timeout = false;   // Enable timeout to stop the motors if no command for some time

// Motor control variables
float motor_speed = 0.0;
float gain_p = 1.0;
float gain_i = 1.0;
float gain_d = 1.0;

void setup() {
    Wire.begin(I2C_ADDRESS);      // join i2c bus
    Wire.onReceive(receiveEvent); // register callback for when we receive data
    Serial.begin(9600);
}

void loop() {
    delay(100);
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