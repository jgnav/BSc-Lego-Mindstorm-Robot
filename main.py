#!/usr/bin/env python3

from robot import *
from ev3dev2.motor import OUTPUT_A, OUTPUT_B
from ev3dev2.sensor import INPUT_1, INPUT_4
from ev3dev2.stopwatch import StopWatch
from ev3dev2.sound import Sound
from time import sleep
import numpy as np
from math import pi
from ev3dev2.button import Button

sound = Sound()
btn = Button()

posicion_inicial = np.array([[0.0], [0.0], [0.0], [pi/2]])

m = motores(motor_izquierdo = OUTPUT_B, motor_derecho = OUTPUT_A, radio_rueda = 0.028, separacion_ruedas = 0.120)
s = sensores_y_bateria(sonar = INPUT_1, sensor_color = INPUT_4)
o = localizacion(motor_izquierdo = OUTPUT_B, motor_derecho = OUTPUT_A, radio_rueda = 0.028, separacion_ruedas = 0.120, posicion = posicion_inicial, loc_sonar = INPUT_1, loc_sensor_color = INPUT_4)
n = navegacion(motor_izquierdo = OUTPUT_B, motor_derecho = OUTPUT_A, radio_rueda = 0.028, separacion_ruedas = 0.120, posicion = posicion_inicial, nav_sonar = INPUT_1, nav_sensor_color = INPUT_4)

KW = 1.5
l = 0.5
waypoints = np.array([[0, l, 0.0], 
                      [l, l, 0.0], 
                      [l, 2*l, 0.0], 
                      [0, 2*l, 0.0],
                      [0, l, 0.0],
                      [l, l, 0.0],
                      [l, 0, 0.0],
                      [0, 0, 0.0]])
n.navegacion_planificada(waypoints, KW)


"""
KA = 1
KR = 10
offxs = -0.09
offys = 0
offphis = 0
rTs = np.array([[cos(offphis), -sin(offphis), 0.0, offxs],
                [sin(offphis), cos(offphis), 0.0, offys],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0]])
punto_destino = np.array([[1.0], [1.0], [0.0]])
n.navegacion_reactiva_campos_virtuales(punto_destino, rTs, KA, KR)
"""

"""
mapa = np.array([[0, 1.03, 0, 0],
                 [1.03, 1.03, 0, 1.03],
                 [1.03, 0, 1.03, 1],
                 [0 ,0 ,1.03, 0]])
rox = 0.05
roy = 0.05
rotheta = 0.573
Rk = 0.1**2
mu, sigma = self.localizacion_probabilistica(mapa, rox, roy, rotheta, Rk)
self.f.write(str(mu[0][0])+" "+str(mu[1][0])+" "+str(mu[2][0])+" "+str(sigma[0][0])+" "+str(sigma[0][1])+" "+str(sigma[0][2])+" "+str(sigma[1][0])+" "+str(sigma[1][1])+" "+str(sigma[1][2])+" "+str(sigma[2][0])+" "+str(sigma[2][1])+" "+str(sigma[2][2])+" "+str(self.posicion_robot[0][0])+" "+str(self.posicion_robot[1][0])+" "+str(self.posicion_robot[2][0])+" "+str(self.posicion_robot[3][0])+"\n")
"""  
"""       
pose = self.odometria("euler")
self.f.write(str(pose[0][0])+" "+str(pose[1][0])+" "+str(pose[2][0])+" "+str(pose[3][0])+"\n")
"""
"""
KW = 2
puntos_destinos = np.array([[0.3, 0.0, 0.0], 
                            [0.3, 0.3, 0.0], 
                            [0.0, 0.3, 0.0], 
                            [0.0, 0.0, 0.0]])
n.navegacion_planificada(puntos_destinos, KW)
"""
"""
KA = 1
KR = 5
punto_destino = np.array([[-1.0], [1.0], [0.0]])
n.navegacion_reactiva_campos_virtuales(punto_destino, KA, KR)
"""

"""
sound.beep()

f = open("log_angulos_izq.txt","w")
angulo = 0

for i in range(0, 12):
    btn.wait_for_bump('left')
    f.write(str(angulo)+" "+str(m.posicion_motor_izquierdo)+"\n")
    angulo = angulo + (pi/6)
    sound.beep()

f.close()
"""

"""
f = open("log_descarga.txt","w")
t = StopWatch()
t.start()
while(1):
    f.write(str(t.value_ms)+" "+str(s.voltaje_bateria)+" "+str(s.corriente_bateria)+"\n")
    sleep(1)
t.stop()
f.close()
"""
"""
o.empezar_posicion_fichero("log_odometria.txt")
for i in range(0, 32): 
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