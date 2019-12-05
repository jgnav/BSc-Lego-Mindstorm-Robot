#!/usr/bin/env python3

from robot import *
from ev3dev2.motor import OUTPUT_A, OUTPUT_B
from ev3dev2.sensor import INPUT_1, INPUT_4
from ev3dev2.stopwatch import StopWatch
from time import sleep
import numpy as np

r = movement(OUTPUT_B, OUTPUT_A, 0.056, 0.114)
s = sensors_and_battery(INPUT_1, INPUT_4)
o = odometry(OUTPUT_B, OUTPUT_A, 0.056, 0.114)

f=open("puntos.txt","w")

s.calibrate_white()
f.write(str(s.color)+" "+str(s.color_name)+" "+str(s.ambient)+" "+str(s.reflection)+" "+str(s.rgb)+"\n")
sleep(0.01)

axis = np.array([[1, 0, 0],
                 [0, 1, 0],
                 [0, 0, 1]])
zero = np.array([1, 0, 0])
o.odometry_start(zero, axis)
o.odometry_stop

f.close()
