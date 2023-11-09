#!/usr/bin/env python3

import sys

if len(sys.argv) < 2:
    print("Give filename!")
    exit()
steps = 100
if len(sys.argv) >= 3:
    steps = int(sys.argv[2])
with open(sys.argv[1],"r") as din:
    inl = []
    while True:
        line = din.readline()
        if not line: break
        parts = line.strip().split(',')
        inl.append([int(x) for x in parts])

step = 0
outl = []
first = 0
for i in range(1,len(inl)):    
    if step == steps-1:
        outl.append([x//steps for x in inl[first]])
        step = 0
    elif step == 0:
        first = i
        step += 1
    else:
        inl[first] = [(x+y) for (x,y) in zip(inl[first],inl[i])]
        step += 1

if step != 0:
    outl.append([x//step for x in inl[first]])

for i in range(len(outl)):
    for j in range(len(outl[i])):
        if j == len(outl[i])-1:
            print(outl[i][j], sep='')
        else:
            print(outl[i][j], ", ", end='', sep='')

        
            
    
