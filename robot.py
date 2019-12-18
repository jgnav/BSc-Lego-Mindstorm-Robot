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
    def __init__(self, motor_izquierdo, motor_derecho, diametro_rueda, separacion_ruedas):
        movimiento.__init__(self, motor_izquierdo, motor_derecho, diametro_rueda, separacion_ruedas)
        self._perimetro_rueda = 2*pi*self._radio
        self._f = None
        self._odometria_activa = False
        self._fin_odometria = True
        self._escribir_fichero_activo = False
        self._fin_escribir_fichero = True
        self._posicion_robot = [0.0, 0.0, 0.0, 0.0]

    def empezar_odometria(self, posicion, modo, tiempo_espera):
        self._posicion_robot = posicion

        def _hilo_odometria():
            izquierda_anterior = 0.0
            derecha_anterior = 0.0
            tiempo_anterior = time()

            while self._odometria_activa:

                izquierda_actual = self._motor_izquierdo.position
                derecha_actual = self._motor_derecho.position
                tiempo_actual = time()

                ticks_izquierda = izquierda_actual - izquierda_anterior
                ticks_derecha = derecha_actual - derecha_anterior
                h = tiempo_actual - tiempo_anterior

                if not ticks_izquierda and not ticks_derecha or not h:
                    if tiempo_espera:
                        sleep(tiempo_espera)
                    continue

                izquierda_anterior = izquierda_actual
                derecha_anterior = derecha_actual
                tiempo_anterior = tiempo_actual

                rotacion_izquierda = float(ticks_izquierda / self._motor_izquierdo.count_per_rot)
                rotacion_derecha = float(ticks_derecha / self._motor_derecho.count_per_rot)

                distancia_izquierda = float(rotacion_izquierda * self._perimetro_rueda)
                distancia_derecha = float(rotacion_derecha * self._perimetro_rueda)

                distancia_total = (distancia_izquierda + distancia_derecha) / 2.0
                rotacion_total = (distancia_derecha - distancia_izquierda) / self._sruedas

                v = distancia_total / h

                if (modo == "euler"):

                    #Euler
                    self._posicion_robot[0] += distancia_total * cos(self._posicion_robot[3])
                    self._posicion_robot[1] += distancia_total * sin(self._posicion_robot[3])
                    self._posicion_robot[3] += rotacion_total

                elif (modo == "RK_2"):

                    #Runge-Kutta de segundo orden
                    self._posicion_robot[0] += distancia_total * cos(self._posicion_robot[3] + (rotacion_total/2))
                    self._posicion_robot[1] += distancia_total * sin(self._posicion_robot[3] + (rotacion_total/2))
                    self._posicion_robot[3] += rotacion_total

                elif (modo == "RK_4"):

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

                if tiempo_espera:
                    sleep(tiempo_espera)

            self._fin_odometria = True

        self._odometria_activa = True
        self._fin_odometria = False
        self._id_hilo_odometria = Thread(target = _hilo_odometria)
        self._id_hilo_odometria.start()

    def parar_odometria(self):
        self._odometria_activa = False
        if not self._fin_odometria:
            self._id_hilo_odometria.join(timeout=None)

    def empezar_posicion_fichero(self, nombre_fichero, tiempo_espera):
        self._f = open(nombre_fichero,"w")

        def _hilo_fichero():
            i = 0
            while self._escribir_fichero_activo:
                self._f.write(str(i)+ " "+str(self._posicion_robot)+"\n")
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
    def __init__(self, motor_izquierdo, motor_derecho, diametro_rueda, separacion_ruedas):
        odometria.__init__(self, motor_izquierdo, motor_derecho, diametro_rueda, separacion_ruedas)
        self.empezar_odometria(self._posicion_robot, "RK_4", 0.001)
        self._ejes_universales = [[1.0, 0.0, 0.0],
                                  [0.0, 1.0, 0.0],
                                  [0.0, 0.0, 1.0]]

    def _theta_a_ejes(self, theta):
        ejes = [[1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0]]

        ejes[0][0] = cos(theta - (pi/2))
        ejes[1][0] = sin(theta - (pi/2))
        ejes[0][1] = cos(theta)
        ejes[1][1] = sin(theta)

        return ejes

    def _matriz_de_rotacion(self, ejesA, ejesB):
        axisA = np.array(ejesA)
        axisB = np.array(ejesB)
        bRa = np.array([[(axisA[:, 0] @ axisB[:, 0]), (axisA[:, 1] @ axisB[:, 0]), (axisA[:, 2] @ axisB[:, 0])],
                        [(axisA[:, 0] @ axisB[:, 1]), (axisA[:, 1] @ axisB[:, 1]), (axisA[:, 2] @ axisB[:, 1])],
                        [(axisA[:, 0] @ axisB[:, 2]), (axisA[:, 1] @ axisB[:, 2]), (axisA[:, 2] @ axisB[:, 2])]])
        return bRa.tolist()


    def _matriz_de_translacion(self, cero, m_rotacion):
        bPa0 = np.array(cero)
        bRa = np.array(m_rotacion)
        bTa = np.array([[bRa[0][0], bRa[0][1], bRa[0][2], bPa0[0]],
                        [bRa[1][0], bRa[1][1], bRa[1][2], bPa0[1]],
                        [bRa[2][0], bRa[2][1], bRa[2][2], bPa0[2]],
                        [0, 0, 0, 1]])
        return bTa.tolist()

    def _matriz_de_translacion_inversa(self, cero, m_rotacion):
        bPa0 = np.array(cero)
        bRa = np.array(m_rotacion)
        bRaT = np.transpose(bRa)
        aux = -(bRaT @ bPa0)
        aTb = np.array([[bRaT[0][0], bRaT[0][1], bRaT[0][2], aux[0]],
                        [bRaT[1][0], bRaT[1][1], bRaT[1][2], aux[1]],
                        [bRaT[2][0], bRaT[2][1], bRaT[2][2], aux[2]],
                        [0, 0, 0, 1]])
        return aTb.tolist()

    def _translacion_de_punto(self, aP, aPb0, axisA, axisB):
        bRa = self._matriz_de_rotacion(axisA, axisB)
        aTb = self._matriz_de_translacion_inversa(aPb0, bRa)
        aPprima = np.append(np.array(aP), 1)
        bP = np.array(aTb) @ aPprima
        return bP[:3].tolist()

    def _c_globales_a_robot(self, coordenadas):
        ejes_robot = self._theta_a_ejes(self._posicion_robot[3])
        return self._translacion_de_punto(coordenadas, self._posicion_robot[:3], self._ejes_universales, ejes_robot)

    def navegacion_reactiva_campos(self, posicion_inicial, posicion_destino):
        self._posicion_robot = posicion_inicial
        v_max = 0.2  #0.5
        w_max = pi/2  #2*pi
        vector_resultado = [0.0, 0.0, 0.0]

        while 1:
            posicion_destino_robot = self._c_globales_a_robot(posicion_destino)
            vector_resultado[0] = posicion_destino_robot[0]
            vector_resultado[1] = posicion_destino_robot[1]

            total = abs(vector_resultado[0]) + abs(vector_resultado[1])
            porcentaje_v = vector_resultado[0] / total
            porcentaje_w = vector_resultado[1] / total
            self.correr(v_max*porcentaje_v, w_max*porcentaje_w)

        self.parar()
