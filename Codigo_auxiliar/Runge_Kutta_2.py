#!/usr/bin/env python3

from ev3dev2.motor import Motor, SpeedRPS, MoveTank
from ev3dev2.sensor.lego import ColorSensor, UltrasonicSensor
from ev3dev2.power import PowerSupply
from math import pi, sin, cos, atan
from threading import Thread
import numpy as np
from time import sleep, time

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
    def __init__(self, motor_izquierdo, motor_derecho, radio_rueda, separacion_ruedas):
        self._motor_izquierdo = Motor(motor_izquierdo)
        self._motor_derecho = Motor(motor_derecho)
        self._dos_motores = MoveTank(motor_izquierdo, motor_derecho)
        self._radio = radio_rueda
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
        self._derecha = ((linear)+((angular*self._sruedas)/2))/self._radio
        self._izquierda = ((linear)-((angular*self._sruedas)/2))/self._radio
        self._dos_motores.on(self._SpeedRadPS(self._izquierda), self._SpeedRadPS(self._derecha))

    def correr_tiempo(self, linear, angular, seconds, bloqueo):
        self._derecha = ((linear)+((angular*self._sruedas)/2))/self._radio
        self._izquierda = ((linear)-((angular*self._sruedas)/2))/self._radio
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
    def __init__(self, motor_izquierdo, motor_derecho, radio_rueda, separacion_ruedas):
        movimiento.__init__(self, motor_izquierdo, motor_derecho, radio_rueda, separacion_ruedas)
        self._tiempo_espera = 0.01
        self._posicion_robot = [0.0, 0.0, 0.0, 0.0]
        self._odometria_activa = False
        self._escribir_fichero_activo = False
        self._f = None

    def empezar_odometria(self, posicion):
        self._posicion_robot = posicion

        def _hilo_odometria():
            izquierda_anterior = 0.0
            derecha_anterior = 0.0

            while self._odometria_activa:

                izquierda_actual = self._motor_izquierdo.position
                derecha_actual = self._motor_derecho.position

                ticks_izquierda = izquierda_actual - izquierda_anterior
                ticks_derecha = derecha_actual - derecha_anterior

                if not ticks_izquierda and not ticks_derecha:
                    if self._tiempo_espera:
                        sleep(self._tiempo_espera)
                    continue

                izquierda_anterior = izquierda_actual
                derecha_anterior = derecha_actual

                rotacion_izquierda = float(ticks_izquierda / self._motor_izquierdo.count_per_rot)
                rotacion_derecha = float(ticks_derecha / self._motor_derecho.count_per_rot)

                distancia_izquierda = float(rotacion_izquierda * self._perimetro_rueda)
                distancia_derecha = float(rotacion_derecha * self._perimetro_rueda)

                distancia_total = (distancia_izquierda + distancia_derecha) / 2.0
                rotacion_total = (distancia_derecha - distancia_izquierda) / self._sruedas

                self._posicion_robot[0] += distancia_total * cos(self._posicion_robot[3] + (rotacion_total/2))
                self._posicion_robot[1] += distancia_total * sin(self._posicion_robot[3] + (rotacion_total/2))
                self._posicion_robot[3] += rotacion_total

                if self._tiempo_espera:
                    sleep(self._tiempo_espera)

        self._odometria_activa = True
        self._id_hilo_odometria = Thread(target = _hilo_odometria)
        self._id_hilo_odometria.start()

    def parar_odometria(self):
        self._odometria_activa = False
        self._id_hilo_odometria.join(timeout=None)

    @property
    def posicion(self):
        return self._posicion_robot

    def empezar_posicion_fichero(self, nombre_fichero):
        self._f = open(nombre_fichero,"w")

        def _hilo_fichero():
            i = 0
            while self._escribir_fichero_activo:
                self._f.write(str(i)+ " "+str(self._posicion_robot)+"\n")
                i = i + 1
                sleep(self._tiempo_espera)

        self._escribir_fichero_activo = True
        self._id_hilo_fichero = Thread(target = _hilo_fichero)
        self._id_hilo_fichero.start()

    def parar_posicion_fichero(self):
        self._escribir_fichero_activo = False
        self._id_hilo_fichero.join(timeout=None)
        self._f.close()











