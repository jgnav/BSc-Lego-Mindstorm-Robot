#!/usr/bin/env python3

from Codigo_auxiliar.euler import *
from ev3dev2.motor import OUTPUT_A, OUTPUT_B
from ev3dev2.sensor import INPUT_1, INPUT_4
from ev3dev2.stopwatch import StopWatch
from time import sleep
import numpy as np
from math import pi

m = movimiento(OUTPUT_B, OUTPUT_A, 0.056, 0.122)
#s = sensores_y_bateria(INPUT_1, INPUT_4)
o = odometria(OUTPUT_B, OUTPUT_A, 0.056, 0.122)

posicion = [0.0, 0.0, 0.0, pi/2]

o.empezar_odometria(posicion, "RK_4")

o.empezar_posicion_fichero("puntos.txt")
for i in range(0,16):
    m.correr_tiempo(0.2, 0, 1.5, True)
    m.correr_tiempo(0, pi/2, 0.89, True)
o.parar_posicion_fichero()

"""
m.correr_tiempo(0.2, 0, 5, True)
f = open("puntos.txt","w")
f.write(str(o.posicion)+"\n")
f.close()
"""
o.parar_odometria()

