#!/usr/bin/env python
import sys

errlist = [ "I2C Error", "MPU Error", "MPUID Error", "MPU Start Error", "State Confusion Error", "Battery Error", "Unknown Error" ] 

def conv(bytes):
    global data
    num = 0
    val = 1
    for x in range(bytes):
        num += int("0x" + data[:2], 16)*val
        val *= 256
        data = data[2:]
    return num

if len(sys.argv) != 2:
    print("Incorrect number of arguments")
    exit()
with open(sys.argv[1]) as f:
    lines = f.readlines()
data = lines[0][9:57]
print "Minutes= ", conv(4)/60
print "Visits = ", conv(2)
print "Succ   = ", conv(2)
print "Volt   = ", conv(2)
for e in errlist:
    cnt = conv(2)
    if cnt != 0:
        print e, "=", cnt
        

