#!/usr/bin/env python
import struct
import serial
from time import sleep
import math
from pprint import pprint
import os

#SMORA = 'L'
SMORA = 'XL'

def retrieve_online_samples():
    print "* Sending bytes..."
    ser.write(struct.pack(">BHH", 0xFE, 200, 200))
    print "* Waiting for answer..."
    fd = open('statistics_online.txt', "w+")
    fd.write('sep=;\nPWM;samples;voltage,current;velocity;\n')
    while 1:
        # pwm, samples, voltage, current, bus_voltage, velocity
        data = ser.readline().strip()
        if 'done' in data:
            break
        data = data.split(',')
        pwm, samples = int(data[0]), int(data[1])
        voltage, current, bus_voltage, velocity = float(data[2]), float(data[3]), float(data[4]), float(data[5])
        fd.write('%d;%d;%.2f;%.2f;%.2f\n' % (pwm, samples, voltage, current, velocity))
        print data
    fd.close()

if SMORA == 'L':
    print "* Opening port..."
    ser = serial.Serial('/dev/cu.wchusbserial1410', 115200)
    print "* Waiting for device..."
    while 1:
        data = ser.readline().strip()
        if data == '* Ready!':
            break
elif SMORA == 'XL':
    print "* Opening port..."
    ser = serial.Serial('/dev/cu.usbmodem1421', 115200)

# get SMORA's online calculation for current, voltage and velocity
retrieve_online_samples()

ser.close()
print "* Done."







