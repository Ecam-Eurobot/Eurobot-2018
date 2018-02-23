# PID gain tuning for motors

The code in this directory allows to measure the step response
of a motor and compute good values for the PID gains.

## Requirements
- Python
    - pyserial: `pip install pyserial`
    - scipy
- Arduino
    - `Encoder` library: can be installed through the Arduino IDE
    - [`FlexiTimer2` library](https://playground.arduino.cc/Main/FlexiTimer2): can be installed from .zip file found at the linked page
- Matlab

## Circuit
### Arduino
- Connect the encoder to pin `2` and `3`
- Connect pin `4` to the direction of the motor driver
- Connect pin `5` to the PWM of the motor driver 

Connect the motor appropriately to the motor driver and 
connect the Arduino to the computer through USB.

## Procedure
1. Upload the `acquire.ino` code to the Arduino
2. Run `acquire.py` to perform a measurement of the step
   response of the motor and recuperate the data from the Arduino.
   You can give a parameter to the script to change the sampling
   time in ms, set to 10 ms by default. If you change it, make
   sure that the Arduino can handle the load.
3. Run `pidTuner` in Matlab...