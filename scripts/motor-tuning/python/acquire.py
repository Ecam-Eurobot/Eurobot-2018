import serial
import sys
import time
import statistics
import datetime
import scipy.io

if len(sys.argv) < 2:
    print("You need to provide a serial port for the Arduino. For example:")
    print("    python acquire.py \"/dev/ttyACM0\"")
    sys.exit()

serial_port = sys.argv[1]
sample_time = sys.argv[2] if len(sys.argv) > 2 else 10

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
    time.sleep(sample_time / 1000)
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
        sd = statistics.stdev(data['output'][:-30])
        trange = max_value - min_value
        if sd <= 0.01 * trange:
            break

scipy.io.savemat('../data/' + datetime.datetime.now().strftime("%Y-%m-%d-%H%M%S") + '.mat', mdict={
    'time': data['time'],
    'input': data['input'],
    'output': data['output']
})
    
