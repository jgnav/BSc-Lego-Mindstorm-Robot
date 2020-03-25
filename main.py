#!/usr/bin/env python3

from robot import *
from ev3dev2.motor import OUTPUT_A, OUTPUT_B
from ev3dev2.sensor import INPUT_1, INPUT_4
from ev3dev2.stopwatch import StopWatch
from time import sleep
import numpy as np
from math import pi

posicion_inicial = np.array([[0.30],
                             [0.30],
                             [0.0],
                             [pi/2]])
punto_destino = np.array([[1.0], [-1.0], [0.0]])
puntos_destinos = np.array([[0.0, 1.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.0]])

m = movimiento(motor_izquierdo = OUTPUT_B, motor_derecho = OUTPUT_A, diametro_rueda = 0.056, separacion_ruedas = 0.122)
s = sensores_y_bateria(INPUT_1, INPUT_4)
o = localizacion(motor_izquierdo = OUTPUT_B, motor_derecho = OUTPUT_A, diametro_rueda = 0.056, separacion_ruedas = 0.122, posicion = posicion_inicial)
n = navegacion(motor_izquierdo = OUTPUT_B, motor_derecho = OUTPUT_A, diametro_rueda = 0.056, separacion_ruedas = 0.122, posicion = posicion_inicial)

#n.navegacion_planificada(puntos_destinos)
#n.navegacion_reactiva_campos_virtuales(punto_destino)


o.empezar_posicion_fichero("puntos.txt")
for i in range(0, 16): #16
    m.correr_tiempo(0.1, 0, 3, True)
    m.correr_tiempo(0, -pi/4, 2, True)
o.parar_posicion_fichero()

