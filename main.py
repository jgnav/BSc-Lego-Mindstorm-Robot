#!/usr/bin/env python3

from clases import robot
from ev3dev2.motor import OUTPUT_A, OUTPUT_B
from ev3dev2.sensor import INPUT_1, INPUT_4
from ev3dev2.stopwatch import StopWatch
from time import sleep

r = robot(OUTPUT_B, OUTPUT_A, INPUT_1, INPUT_4, 0.056, 0.114)

temp = StopWatch()

t = 5000

f=open("puntos.txt","w")

temp.reset()
temp.start()

r.rightMotor_dc = 50
r.leftMotor_dc = -50

sleep(1)

r.stop()

sleep(1)

r.rightMotor_dc = 50
r.leftMotor_dc = -50

sleep(1)

r.stop
temp.stop()
f.close()
