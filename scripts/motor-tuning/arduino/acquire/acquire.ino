#include <Encoder.h>
#include <FlexiTimer2.h>

// Defines the pins to control the motor driver
// pin 5 conrols the voltage to the motor by a PWM
// pin 4 controls the direction of the motor
const int PWM = 5;
const int DIR = 4;

Encoder motor_encoder = Encoder(2, 3);
const int ENCODER_TICKS_PER_REV = 3200;

// Defines the time between two samples in miliseconds
int sampling_time = 10;
double dt = sampling_time / 1000.;

// Input is a value to drive the PWM signals
// Output is a value representing the angular velocity of the motor
int input = 0;
volatile double t0 = 0;
volatile double omega = 0;

// Variable to keep track of the old encoder value
int old_encoder = 0;

void setup() {
    Serial.begin(19200);

    // Configure pins
    pinMode(DIR, OUTPUT);
    pinMode(PWM, OUTPUT);
    
    // Set the motor direction and set the velocity to zero
    digitalWrite(DIR, LOW);
    analogWrite(PWM, input);

    Serial.println("Waiting for sampling time: ");
    // The Arduino will wait for input on the Serial bus
    while(Serial.available() == 0) {}
    
    sampling_time = Serial.read();
    dt = sampling_time / 1000.;

    // Register a callback that is executed precisely
    // every SAMPLING_TIME to compute the angular velocity
    // of the motor and send it to the serial bus.
    FlexiTimer2::set(sampling_time, isrt);
    FlexiTimer2::start();
    t0 = micros();
}

void loop() {

    // After 30 samples, apply the step by setting the PWM value to max
    double t = (micros() - t0) / 1000000.;
    if (t > 30*dt) {
        input = 255;
    }
    
    analogWrite(PWM, input);
    delay(sampling_time / 2);
}


// Function that is called periodically every sample_time
// to compute the angular velocity of the motor
void isrt(){
    int deltaEncoder = motor_encoder.read() - old_encoder;
    old_encoder = motor_encoder.read();

    // Angular velocity in rad/s
    omega = ((2. * 3.141592 * ((double)deltaEncoder)) / ENCODER_TICKS_PER_REV) / dt;
    double t = (micros() - t0) / 1000000.;

    if (isnan(omega)) {
        Serial.println("Uninitialized...");
    } else {
        Serial.print(t);
        Serial.print(",");
        Serial.print(input);
        Serial.print(",");
        Serial.print(omega);
        Serial.println();
    }
}



