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

def set_PIV_parameters(Kp, Ki, Kv, frequency, speed):
    print "* Sending bytes..."
    ser.write(struct.pack("<BfffHH", 0xF0, float(Kp), float(Ki), float(Kv), frequency, speed))
    print "* Waiting for answer..."
    while 1:
        data = ser.readline().strip()
        if 'done' in data:
            break
    print "* Done"

def retrieve_PIV_samples(initAngle, finalAngle, duration):
    print "* Sending bytes..."
    ser.write(struct.pack("<BffH", 0xF1, initAngle, finalAngle, duration))
    print "* Waiting for answer..."
    while 1:
        # currentMillis, desired_angle, current_angle, pivOutput, temperature
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
            'pivOutput': pidOutput,
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
# get SMORA's PIV output
"""
BW = 20.0     # bandwidth - Hz
damping = 1.0 # damping ratio
J = 1.0       # rotor and load moment of inertia
b = 1.0       # motor's damping constant
Kp = (2*math.pi*BW)/(2*damping+1)
Ki = ((2*math.pi*BW)**2)*(2*damping+1)*J
Kv = (2*math.pi*BW)*((2*damping+1)*J) - b
print "Kp: %.5f, Ki: %.5f, Kv: %.5f" % (Kp, Ki, Kv)
exit(0)
"""

Kp = 10
Ki = 0.3
Kv = 5

speed = 100
frequency = 100
initAngle = 120.0
finalAngle = 180.0
duration = 2000
set_PIV_parameters(Kp, Ki, Kv, frequency, speed)
retrieve_PIV_samples(initAngle, finalAngle, duration)

time = []
goalAngle = []
currentAngle = []
pivOutput = []
temperature = []
for sample in buffer:
    time.append(sample['time'])
    goalAngle.append(sample['goalAngle'])
    currentAngle.append(sample['currentAngle'])
    pivOutput.append(sample['pivOutput'])
    temperature.append(sample['temperature'])

for index in range(1,len(time)):
    time[index] = time[index] - time[0]
time[0] = 0

title = "Kp=%.3f, Ki=%.3f, Kv=%.3f, F=%dHz, S=%s, T=%.3f, d=%d, angle0=%.2f, angle1=%.2f" % (Kp, Ki, Kv, frequency, speed, temperature[-1], duration, initAngle, finalAngle)
sci.xtitle(title)
sci.subplot(2,1,1);
sci.plot(time, goalAngle, 'b')
sci.plot(time, currentAngle, 'r')
sci.subplot(2,1,2);
sci.plot(time, pivOutput, 'g')
#sci.plot(time, temperature)
#sci.xs2pdf(0, '%s.pdf'%title)
raw_input("Press Enter to continue...")
sci.close()

ser.close()
print "* Done."







