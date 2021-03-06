#!/usr/bin/env python
import struct
import serial
from time import sleep
import math
from pprint import pprint
import os
os.environ["PATH"] += os.pathsep + "/Applications/scilab-5.5.2.app/Contents/MacOS/bin"
from scilab2py import Scilab2Py
sci = Scilab2Py()

#SMORA = 'L'
SMORA = 'XL'

def set_PID_parameters(Kp, Ki, Kd, frequency):
    print "* Sending bytes..."
    ser.write(struct.pack("<BfffH", 0xFB, float(Kp), float(Ki), float(Kd), frequency))
    print "* Waiting for answer..."
    while 1:
        data = ser.readline().strip()
        if 'done' in data:
            break
    print "* Done"

def retrieve_PID_samples(initAngle, finalAngle, duration):
    print "* Sending bytes..."
    ser.write(struct.pack("<BffH", 0xFA, initAngle, finalAngle, duration))
    print "* Waiting for answer..."
    while 1:
        # currentMillis, desired_angle, current_angle, pidOutput, temperature
        data = ser.readline().strip()
        if 'done' in data:
            break
        data = data.split(',')
        millis = int(data[0])
        goalAngle, currentAngle = float(data[1]), float(data[2])
        pidOutput = float(data[3])
        temperature = float(data[4])
        buffer.append({
            'time': millis,
            'goalAngle': goalAngle,
            'currentAngle': currentAngle,
            'pidOutput': pidOutput,
            'temperature': temperature
        })
        print millis, goalAngle, currentAngle, pidOutput, temperature

if SMORA == 'L':
    print "* Opening port..."
    ser = serial.Serial('/dev/cu.wchusbserial1410', 115200)
    print "* Waiting for device..."
    while 1:
        data = ser.readline().strip()
        if data == '* Ready':
            break
elif SMORA == 'XL':
    print "* Opening port..."
    ser = serial.Serial('/dev/cu.usbmodem1421', 115200)

buffer = []
# get SMORA's PID output
Kp = 1      # 5     # 40   # 60
Ki = 0.2     # 1.1   # 1    # 1.1
Kd = 0.01     # 0     # 1    $ 1.0
frequency = 100
initAngle = 90.0
finalAngle = 180.0
duration = 1000
set_PID_parameters(Kp, Ki, Kd, frequency)
retrieve_PID_samples(initAngle, finalAngle, duration)

time = []
goalAngle = []
currentAngle = []
pidOutput = []
temperature = []
for sample in buffer:
    time.append(sample['time'])
    goalAngle.append(sample['goalAngle'])
    currentAngle.append(sample['currentAngle'])
    pidOutput.append(sample['pidOutput'])
    temperature.append(sample['temperature'])

for index in range(1,len(time)):
    time[index] = time[index] - time[0]
time[0] = 0

title = "Kp=%.3f, Ki=%.3f, Kd=%.3f, F=%dHz, T=%.3f, d=%d, angle0=%.2f, angle1=%.2f" % (Kp, Ki, Kd, frequency, temperature[-1], duration, initAngle, finalAngle)
sci.xtitle(title)
sci.subplot(2,1,1);
sci.plot(time, goalAngle, 'b')
sci.plot(time, currentAngle, 'r')
sci.subplot(2,1,2);
sci.plot(time, pidOutput, 'g')
#sci.plot(time, temperature)
#sci.xs2pdf(0, '%s.pdf'%title)
raw_input("Press Enter to continue...")
sci.close()

ser.close()
print "* Done."







