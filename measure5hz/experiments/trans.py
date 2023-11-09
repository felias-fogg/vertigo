#!/usr/bin/env python3

import serial
import sys

if len(sys.argv) == 2:
    fname = sys.argv[1]
else:
    print("No filename given")
    exit

try:
    with serial.Serial('/dev/tty.usbserial-A400XKI3', 57600, timeout=20) as ser:
        with open(fname, "w") as file:
            line = ""
            while line != "END":
                line = str(ser.readline(),"utf-8").strip()
                if line[0] < '0' or line[0] > '9': continue
                print(line, file=file)
except serial.serialutil.SerialException:
    print("Cannot open serial interface /dev/tty.usbserial-A400XKI3");
    exit()
except KeyboardInterrupt:
    print("\nTerminated!")
    exit()

file.close()
