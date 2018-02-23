import serial
import sys
import argparse
import time
import statistics
import datetime
import scipy.io

# Command line argument parser
desc = 'This script allows to acquire data representing a step response from a motor through an Arduino'

parser = argparse.ArgumentParser(description=desc)
parser.add_argument('serial', nargs=1, help="Specify the serial port of the Arduino")
parser.add_argument('-o', '--output', nargs=1, help="Secifies the destination of the output file")
parser.add_argument('--sampling', nargs=1, help="Changes the time between samples in ms")

args = parser.parse_args() 

serial_port = args.serial[0]
sampling_time = args.sampling[0] if args.sampling else 10
dest = args.output[0] if args.output else '../data/' + datetime.datetime.now().strftime("%Y-%m-%d-%H%M%S")

try:
    arduino = serial.Serial(serial_port, timeout=1, baudrate=19200)
except:
    print("Could not connect to the serial port...")
    sys.exit()

# discard the first line from the Arduino requesting input
while b'Waiting for sampling time' not in arduino.readline():
    pass

time.sleep(0.1)

arduino.write(b'10')

data = {
    'input': [],
    'output': [],
    'time': [],
}

min_value = 1000
max_value = 0

while True:
    time.sleep(sampling_time / 1000)
    received = arduino.readline().strip(b'\n').strip(b'\r').split(b',')

    t = float(received[0])
    i = int(received[1])
    o = float(received[2])

    data['time'].append(t)
    data['input'].append(i)
    data['output'].append(o)

    min_value = min(min_value, o)
    max_value = max(max_value, o)

    # If we have more than 60 samples, check if the system
    # has stabilised by computing the standard deviation
    # of the last 30 samples compared to the total range
    if len(data['output']) > 60:
        sd = statistics.stdev(data['output'][-30:])
        trange = max_value - min_value
        print(str(sd) + ", " + str(trange))
        if sd <= 0.01 * trange:
            break

scipy.io.savemat(dest + '.mat', mdict={
    'time': data['time'],
    'input': data['input'],
    'output': data['output']
})
    
