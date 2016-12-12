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

def set_PID_parameters(Kp, Ki, Kd, Kf, frequency):
    print "* Sending bytes..."
    ser.write(struct.pack("<BffffH", 0xFB, float(Kp), float(Ki), float(Kd), float(Kf), frequency))
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
    count = 0
    while 1:
        # millis, goalSpeed, currentSpeed, pidOutput, temperature, cycle_duration
        data = ser.readline().strip()
        if 'done' in data:
            break
        data = data.split(',')
        millis = int(data[0])
        goalSpeed, currentSpeed = float(data[1]), float(data[2])
        pidOutput = float(data[3])
        temperature = float(data[4])
        cycle_duration = int(data[5])
        dtMicros = int(data[6])
        voltage, current = float(data[7]), float(data[8])
        buffer.append({
            'time': millis,
            'goalSpeed': goalSpeed,
            'currentSpeed': currentSpeed,
            'pidOutput': pidOutput,
            'temperature': temperature,
            'cycle_duration': cycle_duration,
            'voltage': voltage,
            'current': current,
        })
        print count, '\t', millis, '\t', goalSpeed, '\t', currentSpeed, '\t', pidOutput, '\t', temperature, '\t', cycle_duration/1000.0, 'ms', '\t', dtMicros, 'us', '\t', current, 'mA \t', voltage, 'V \t'
        count += 1

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
    #ser = serial.Serial('/dev/cu.usbmodem1421', 115200)
    ser = serial.Serial('/dev/cu.usbmodem1421', 2500000)

buffer = []
# get SMORA's PID output
Kp = 0       # 0.003
Ki = 0       # 0.005
Kd = 0           #
Kf = 0.02        # 0.02
frequency = 100
initSpeed = 200.0
finalSpeed = 150.0
duration = 1000
set_PID_parameters(Kp, Ki, Kd, Kf, frequency)
retrieve_PID_samples(initSpeed, finalSpeed, duration)

time = []
goalSpeed = []
currentSpeed = []
pidOutput = []
temperature = []
current = []
for sample in buffer:
    time.append(sample['time'])
    goalSpeed.append(sample['goalSpeed'])
    currentSpeed.append(sample['currentSpeed'])
    pidOutput.append(sample['pidOutput'])
    temperature.append(sample['temperature'])
    current.append(sample['current'])

for index in range(1,len(time)):
    time[index] = time[index] - time[0]
time[0] = 0

title = "Kp=%.3f, Ki=%.3f, Kd=%.3f, F=%dHz, T=%.3f, d=%d, speed0=%.2f, speed1=%.2f" % (Kp, Ki, Kd, frequency, temperature[-1], duration, initSpeed, finalSpeed)
sci.xtitle(title)
sci.subplot(3,1,1);
sci.plot(time, goalSpeed, 'b')
sci.plot(time, currentSpeed, 'r')
sci.subplot(3,1,2);
sci.plot(time, pidOutput, 'g')
sci.subplot(3,1,3);
sci.plot(time, current, 'b')
#sci.plot(time, temperature)
#sci.xs2pdf(0, '%s.pdf'%title)
raw_input("Press Enter to continue...")
sci.close()

ser.close()
print "* Done."







