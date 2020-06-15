#!/usr/bin/env python3

from robot import *
from ev3dev2.motor import OUTPUT_A, OUTPUT_B
from ev3dev2.sensor import INPUT_1, INPUT_4
from ev3dev2.stopwatch import StopWatch
from ev3dev2.sound import Sound
from time import sleep
import numpy as np
from math import pi

skpr = Sound()

posicion_inicial = np.array([[0.30], [0.30], [0.0], [pi/2]])
punto_destino = np.array([[1.0], [-1.0], [0.0]])
puntos_destinos = np.array([[0.0, 1.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.0]])

m = motores(motor_izquierdo = OUTPUT_B, motor_derecho = OUTPUT_A, diametro_rueda = 0.056, separacion_ruedas = 0.120)
s = sensores_y_bateria(sonar = INPUT_1, sensor_color = INPUT_4)
o = localizacion(motor_izquierdo = OUTPUT_B, motor_derecho = OUTPUT_A, diametro_rueda = 0.056, separacion_ruedas = 0.120, posicion = posicion_inicial, loc_sonar = INPUT_1, loc_sensor_color = INPUT_4)
n = navegacion(motor_izquierdo = OUTPUT_B, motor_derecho = OUTPUT_A, diametro_rueda = 0.056, separacion_ruedas = 0.120, posicion = posicion_inicial, nav_sonar = INPUT_1, nav_sensor_color = INPUT_4)


f = open("log_descarga.txt","w")
t = StopWatch()
t.start()
while(1):
    f.write(str(t.value_ms)+" "+str(s.voltaje_bateria)+" "+str(s.corriente_bateria)+"\n")
    sleep(1)
t.stop()
f.close()

"""
o.empezar_posicion_fichero("log_odometria.txt")
for i in range(0, 1): #16
    m.correr_tiempo(0.08, 0, 3, True)
    m.correr_tiempo(0.047, -pi/4, 1.77, True)
o.parar_posicion_fichero()
"""

"""
n.navegacion_planificada(puntos_destinos)
n.navegacion_reactiva_campos_virtuales(punto_destino)
"""

"""
[0, 0, 0],[25.5, 25.5, 25.5],[51, 51, 51],[76.5, 76.5, 76.5],[25.52, 25.52, 25.52],[127.5, 127.5, 127.5],[153, 153, 153],[178.5, 178.5, 178.5],[204, 204, 204],[229.5, 229.5, 229.5],[242.2, 242.2, 242.2],[255, 0, 0],[255, 25.5, 25.5],[255, 51, 51],[255, 76.5, 76.5],[255, 102.0, 102.0],[255, 127.5, 127.5],[255, 153, 153],[255, 178.5, 178.5],[255, 204, 204],[255, 229.5, 229.5],[255, 242.2, 242.2],[0, 255, 0],[25.5, 255, 25.5],[51, 255, 51],[76.5, 255, 76.5],[102.0, 255, 102.0],[127.5, 255, 127.5],[153, 255, 153],[178.5, 255, 178.5],[204, 255, 204],[229.5, 255, 229.5],[242.2, 255, 242.2],[0, 0, 255],[25.5, 25.5, 255],[51, 51, 255],[76.5, 76.5, 255],[102.0, 102.0, 255],[127.5, 127.5, 255],[153, 153, 255],[178.5, 178.5, 255],[204, 204, 255],[229.5, 229.5, 255],[242.2, 242.2, 255],[255, 255, 255]
colores = [[255, 255, 255],[242.2, 242.2, 242.2],[229.5, 229.5, 229.5],[204.0, 204.0, 204.0],[178.5, 178.5, 178.5],[153, 153, 153],[127.5, 127.5, 127.5],[102.0, 102.0, 102.0],[76.5, 76.5, 76.5],[51.0, 51.0, 51.0],[25.5, 25.5, 25.5],[0, 0, 0],[255, 0, 0],[242.2, 0, 0],[229.5, 0, 0],[204.0, 0, 0],[178.5, 0, 0],[153.0, 0, 0],[127.5, 0, 0],[102.0, 0, 0],[76.5, 0, 0],[51.0, 0, 0],[25.5, 0, 0],[0, 255, 0],[0, 242.2, 0],[0, 229.5, 0],[0, 204.0, 0],[0, 178.5, 0],[0, 153.0, 0],[0, 127.5, 0],[0, 102.0, 0],[0, 76.5, 0],[0, 51.0, 0],[0, 25.5, 0],[0, 0, 255],[0, 0, 242.2],[0, 0, 229.5],[0, 0, 204.0],[0, 0, 178.5],[0, 0, 153.0],[0, 0, 127.5],[0, 0, 102.0],[0, 0, 76.5],[0, 0, 51.0],[0, 0, 25.5]]

s.calibrar_blanco()
skpr.beep()
sleep(10)
f = open("log_color.txt","w")

for color in colores:
    for i in range(0, 100):  
        f.write(str(color)+" "+str(s.rgb)+" "+str(s.color)+" "+str(s.reflexion)+" "+str(s.ambiente)+"\n")
    skpr.beep()
    sleep(3)
f.close()
"""