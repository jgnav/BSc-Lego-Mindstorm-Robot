#!/usr/bin/env python3

from clases import robot
from ev3dev2.motor import OUTPUT_A, OUTPUT_B
from ev3dev2.sensor import INPUT_1, INPUT_4
from ev3dev2.stopwatch import StopWatch
from time import sleep

r = robot(OUTPUT_B, OUTPUT_A, INPUT_1, INPUT_4, 0.056, 0.114)

f=open("puntos.txt","w")

r.calibrate_white()
f.write(str(r.color)+" "+str(r.color_name)+" "+str(r.ambient)+" "+str(r.reflection)+" "+str(r.rgb)+"\n")
sleep(0.01)

f.close()
