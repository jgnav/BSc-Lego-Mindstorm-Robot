#!/usr/bin/env python3

from ev3dev2.motor import Motor, SpeedRPS, MoveTank
from ev3dev2.sensor.lego import ColorSensor, UltrasonicSensor
from ev3dev2.power import PowerSupply
from math import pi, sin, cos, atan
from threading import Thread
import numpy as np
from time import sleep, time
from ev3dev2.sensor import INPUT_1, INPUT_4

class sensores_y_bateria:
    def __init__(self, sonar, sensor_color):
        self._sonar = UltrasonicSensor(sonar)
        self._color = ColorSensor(sensor_color)
        self._bateria = PowerSupply()

    #Bateria
    @property
    def voltaje_bateria(self):
        return self._bateria.measured_volts

    @property
    def correinte_bateria(self):
        return self._bateria.measured_amps

    #Sensor sonar
    @property
    def distancia_sonar(self):
        return (self._sonar.distance_centimeters / 100)

    @property
    def otros_Sensores_presentes(self):
        return self._sonar.other_sensor_present

    #Sensor color
    def calibrar_blaco(self):
        self._color.calibrate_white()

    @property
    def color(self):
        return self._color.color

    @property
    def nombre_color(self):
        return self._color.color_name

    @property
    def ambiente(self):
        return self._color.ambient_light_intensity

    @property
    def reflexion(self):
        return self._color.reflected_light_intensity

    @property
    def rgb(self):
        return self._color.rgb

class movimiento:
    def __init__(self, motor_izquierdo, motor_derecho, diametro_rueda, separacion_ruedas):
        self._motor_izquierdo = Motor(motor_izquierdo)
        self._motor_derecho = Motor(motor_derecho)
        self._dos_motores = MoveTank(motor_izquierdo, motor_derecho)
        self._radio = diametro_rueda/2
        self._sruedas = separacion_ruedas

    def _SpeedRadPS(self, value):
        return SpeedRPS(value/(2*pi))

    #Motores separados
    @property
    def w_motor_derecho(self):
        return 2*pi*(self._motor_derecho.speed/self._motor_derecho.count_per_rot)

    @w_motor_derecho.setter
    def w_motor_derecho(self, velocidad):
        self._motor_derecho.on(self._SpeedRadPS(velocidad))

    @property
    def w_motor_izquierdo(self):
        return 2*pi*(self._motor_izquierdo.speed/self._motor_izquierdo.count_per_rot)

    @w_motor_izquierdo.setter
    def w_motor_izquierdo(self, velocidad):
        self._motor_izquierdo.on(self._SpeedRadPS(velocidad))

    @property
    def dc_motor_izquierdo(self):
        return self._motor_izquierdo.duty_cycle

    @dc_motor_izquierdo.setter
    def dc_motor_izquierdo(self, ciclo):
        self._motor_izquierdo.run_direct(duty_cycle_sp = ciclo)

    @property
    def dc_motor_derecho(self):
        return self._motor_derecho.duty_cycle

    @dc_motor_derecho.setter
    def dc_motor_derecho(self, ciclo):
        self._motor_derecho.run_direct(duty_cycle_sp = ciclo)

    #Ambos motores
    def correr(self, linear, angular):
        self._derecha = ((linear)-((angular*self._sruedas)/2))/self._radio
        self._izquierda = ((linear)+((angular*self._sruedas)/2))/self._radio
        self._dos_motores.on(self._SpeedRadPS(self._izquierda), self._SpeedRadPS(self._derecha))

    def correr_tiempo(self, linear, angular, seconds, bloqueo):
        self._derecha = ((linear)-((angular*self._sruedas)/2))/self._radio
        self._izquierda = ((linear)+((angular*self._sruedas)/2))/self._radio
        self._dos_motores.on_for_seconds(self._SpeedRadPS(self._izquierda), self._SpeedRadPS(self._derecha), seconds, block = bloqueo)

    def parar(self):
        self._dos_motores.off()

    @property
    def velocidad_linear(self):
        return ((self.w_motor_derecho+self.w_motor_izquierdo)/2)*self._radio

    @property
    def velocidad_angular(self):
        return ((self.w_motor_derecho-self.w_motor_izquierdo)*self._radio)/self._sruedas


class odometria(movimiento):
    def __init__(self, motor_izquierdo, motor_derecho, diametro_rueda, separacion_ruedas, posicion, modo):
        movimiento.__init__(self, motor_izquierdo, motor_derecho, diametro_rueda, separacion_ruedas)

        self._perimetro_rueda = 2*pi*self._radio

        self._f = None
        self._escribir_fichero_activo = False
        self._fin_escribir_fichero = True

        self._posicion_robot = posicion
        self._izquierda_anterior = self._motor_izquierdo.position
        self._derecha_anterior = self._motor_derecho.position
        self._tiempo_anterior = time()
        self._modo = modo

    def posicion(self):
        izquierda_actual = self._motor_izquierdo.position
        derecha_actual = self._motor_derecho.position
        tiempo_actual = time()

        ticks_izquierda = izquierda_actual - self._izquierda_anterior
        ticks_derecha = derecha_actual - self._derecha_anterior
        h = tiempo_actual - self._tiempo_anterior

        if ticks_izquierda and ticks_derecha and h:
            self._izquierda_anterior = izquierda_actual
            self._derecha_anterior = derecha_actual
            self._tiempo_anterior = tiempo_actual

            rotacion_izquierda = float(ticks_izquierda / self._motor_izquierdo.count_per_rot)
            rotacion_derecha = float(ticks_derecha / self._motor_derecho.count_per_rot)

            distancia_izquierda = float(rotacion_izquierda * self._perimetro_rueda)
            distancia_derecha = float(rotacion_derecha * self._perimetro_rueda)

            distancia_total = (distancia_izquierda + distancia_derecha) / 2.0
            rotacion_total = (distancia_derecha - distancia_izquierda) / self._sruedas

            v = distancia_total / h

            if (self._modo == "euler"):
                #Euler
                self._posicion_robot[0] += distancia_total * cos(self._posicion_robot[3])
                self._posicion_robot[1] += distancia_total * sin(self._posicion_robot[3])
                self._posicion_robot[3] += rotacion_total

            elif (self._modo == "RK_2"):
                #Runge-Kutta de segundo orden
                self._posicion_robot[0] += distancia_total * cos(self._posicion_robot[3] + (rotacion_total/2))
                self._posicion_robot[1] += distancia_total * sin(self._posicion_robot[3] + (rotacion_total/2))
                self._posicion_robot[3] += rotacion_total

            elif (self._modo == "RK_4"):
                #Runge-Kutta de cuarto orden
                k01 = v * cos(self._posicion_robot[3])
                k02 = (v + 0.5*h) * cos(self._posicion_robot[3] + 0.5*k01*h)
                k03 = (v + 0.5*h) * cos(self._posicion_robot[3] + 0.5*k02*h)
                k04 = (v + h) * cos(self._posicion_robot[3] + k03*h)

                k11 = v * sin(self._posicion_robot[3])
                k12 = (v + 0.5*h) * sin(self._posicion_robot[3] + 0.5*k11*h)
                k13 = (v + 0.5*h) * sin(self._posicion_robot[3] + 0.5*k12*h)
                k14 = (v + h) * sin(self._posicion_robot[3] + k13*h)

                self._posicion_robot[0] += (1/6)*h*(k01 + 2*(k02 + k03) + k04)
                self._posicion_robot[1] += (1/6)*h*(k11 + 2*(k12 + k13) + k14)
                self._posicion_robot[3] += rotacion_total

        return self._posicion_robot


    def empezar_posicion_fichero(self, nombre_fichero, tiempo_espera):
        self._f = open(nombre_fichero,"w")

        def _hilo_fichero():
            i = 0
            while self._escribir_fichero_activo:
                self._f.write(str(i)+ " "+str(self.posicion())+"\n")
                i = i + 1
                sleep(tiempo_espera)

            self._fin_escribir_fichero = True

        self._escribir_fichero_activo = True
        self._fin_escribir_fichero = False
        self._id_hilo_fichero = Thread(target = _hilo_fichero)
        self._id_hilo_fichero.start()

    def parar_posicion_fichero(self):
        self._escribir_fichero_activo = False
        if not self._fin_escribir_fichero:
            self._id_hilo_fichero.join(timeout=None)
        self._f.close()

class navegacion(odometria):
    def __init__(self, motor_izquierdo, motor_derecho, diametro_rueda, separacion_ruedas, posicion, modo):
        odometria.__init__(self, motor_izquierdo, motor_derecho, diametro_rueda, separacion_ruedas, posicion, modo)
        self._s = sensores_y_bateria(INPUT_1, INPUT_4)

    def _coordenadas_global_a_robot(self, posicion_robot, punto_global):
        angulo = posicion_robot[3] - pi/2

        R = np.array([[cos(angulo), -sin(angulo), 0],
                      [sin(angulo), cos(angulo), 0],
                      [0.0, 0.0, 1.0]])

        Rt = np.transpose(R)

        aux = -(Rt @ posicion_robot[:3])
        T = np.array([[Rt[0][0], Rt[0][1], Rt[0][2], aux[0]],
                     [Rt[1][0], Rt[1][1], Rt[1][2], aux[1]],
                     [Rt[2][0], Rt[2][1], Rt[2][2], aux[2]],
                     [0, 0, 0, 1]])

        resultado = T @ np.append(np.array(punto_global), 1)
        return resultado[:3].tolist()

    def navegacion_reactiva_campos_virtuales(self, punto_destino):
        self._f = open("puntos.txt","w") #*******
        i = 0 #*********

        vector_resultante = [0.0, 0.0, 0.0]
        KA = 1.0
        KR = 4.0

        while 1:
            posicion_robot = self.posicion()
            vector_hasta_destino = self._coordenadas_global_a_robot(posicion_robot, punto_destino)
            modulo = np.sqrt(np.array(vector_hasta_destino) @ np.array(vector_hasta_destino))
            if (modulo <= 0.05):
                break

            if (self._s.distancia_sonar - 0.09) > 0.45:
                distancia_obstaculo = 0
            else:
                distancia_obstaculo = self._s.distancia_sonar - 0.09

            vector_resultante[0] = KA*vector_hasta_destino[0]
            vector_resultante[1] = KA*vector_hasta_destino[1] - KR*distancia_obstaculo
            vector_resultante[2] = KA*vector_hasta_destino[2]

            if((0.2 * vector_resultante[1]) > 0.5):
                v = 0.5
            else:
                v = 0.2 * vector_resultante[1]

            if((2 * vector_resultante[0]) > 2*pi):
                w = 2 * pi
            else:
                w = 2 * vector_resultante[0]

            self.correr(v, w)

            self._f.write(str(i)+" "+str(posicion_robot)+str(vector_hasta_destino)+" "+str(modulo)+" "+str(vector_resultante)+" "+str(v)+" "+str(w)+"\n")
            i = i + 1

            sleep(0.01)

        self.parar()
        self._f.close()
