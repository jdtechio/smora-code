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

def retrieve_PID_samples(initSpeed, finalSpeed, duration):
    print "* Sending bytes..."
    ser.write(struct.pack("<BffH", 0xFA, initSpeed, finalSpeed, duration))
    print "* Waiting for answer..."
    while 1:
        # currentMillis, desired_speed, current_speed, pidOutput, temperature
        data = ser.readline().strip()
        if 'done' in data:
            break
        data = data.split(',')
        millis = int(data[0])
        goalSpeed, currentSpeed = float(data[1]), float(data[2])
        pidOutput = float(data[3])
        temperature = float(data[4])
        buffer.append({
            'time': millis,
            'goalSpeed': goalSpeed,
            'currentSpeed': currentSpeed,
            'pidOutput': pidOutput,
            'temperature': temperature
        })
        print millis, goalSpeed, currentSpeed, pidOutput, temperature

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
Kp = 1
Ki = 0
Kd = 0
frequency = 100
initSpeed = 90.0
finalSpeed = 180.0
duration = 2000
set_PID_parameters(Kp, Ki, Kd, frequency)
retrieve_PID_samples(initSpeed, finalSpeed, duration)

time = []
goalSpeed = []
currentSpeed = []
pidOutput = []
temperature = []
for sample in buffer:
    time.append(sample['time'])
    goalSpeed.append(sample['goalSpeed'])
    currentSpeed.append(sample['currentSpeed'])
    pidOutput.append(sample['pidOutput'])
    temperature.append(sample['temperature'])

for index in range(1,len(time)):
    time[index] = time[index] - time[0]
time[0] = 0

title = "Kp=%.3f, Ki=%.3f, Kd=%.3f, F=%dHz, T=%.3f, d=%d, speed0=%.2f, speed1=%.2f" % (Kp, Ki, Kd, frequency, temperature[-1], duration, initSpeed, finalSpeed)
sci.xtitle(title)
sci.subplot(2,1,1);
sci.plot(time, goalSpeed, 'b')
sci.plot(time, currentSpeed, 'r')
sci.subplot(2,1,2);
sci.plot(time, pidOutput, 'g')
#sci.plot(time, temperature)
#sci.xs2pdf(0, '%s.pdf'%title)
raw_input("Press Enter to continue...")
sci.close()

ser.close()
print "* Done."







