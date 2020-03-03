#!/usr/bin/env python3

from ev3dev2.motor import Motor, SpeedRPS, MoveTank
from ev3dev2.sensor.lego import ColorSensor, UltrasonicSensor
from ev3dev2.power import PowerSupply
from math import pi, sin, cos, atan, atan2
from threading import Thread
import numpy as np
from time import sleep, time
from ev3dev2.sensor import INPUT_1, INPUT_4

class sensores_y_bateria:
    def __init__(self, sonar, sensor_suelo):
        self.sonar = UltrasonicSensor(sonar)
        self.suelo = ColorSensor(sensor_suelo)
        self.bateria = PowerSupply()

    #Bateria
    @property
    def voltaje_bateria(self):
        return self.bateria.measured_volts

    @property
    def corriente_bateria(self):
        return self.bateria.measured_amps

    #Sensor sonar
    @property
    def distancia_sonar(self):
        return (self.sonar.distance_centimeters / 100)

    @property
    def otros_Sensores_presentes(self):
        return self.sonar.other_sensor_present

    #Sensor suelo
    def calibrar_blaco(self):
        self.suelo.calibrate_white()

    @property
    def color(self):
        return self.suelo.color

    @property
    def nombre_color(self):
        return self.suelo.color_name

    @property
    def ambiente(self):
        return self.suelo.ambient_light_intensity

    @property
    def reflexion(self):
        return self.suelo.reflected_light_intensity

    @property
    def rgb(self):
        return self.suelo.rgb

class movimiento:
    def __init__(self, motor_izquierdo, motor_derecho, diametro_rueda, separacion_ruedas):
        self.motor_izquierdo = Motor(motor_izquierdo)
        self.motor_derecho = Motor(motor_derecho)
        self.dos_motores = MoveTank(motor_izquierdo, motor_derecho)
        self.radio = diametro_rueda/2
        self.sruedas = separacion_ruedas

    def SpeedRadPS(self, value):
        return SpeedRPS(value/(2*pi))

    #Motores separados
    @property
    def w_motor_derecho(self):
        return 2*pi*(self.motor_derecho.speed/self.motor_derecho.count_per_rot)

    @w_motor_derecho.setter
    def w_motor_derecho(self, velocidad):
        self.motor_derecho.on(self.SpeedRadPS(velocidad))

    @property
    def w_motor_izquierdo(self):
        return 2*pi*(self.motor_izquierdo.speed/self.motor_izquierdo.count_per_rot)

    @w_motor_izquierdo.setter
    def w_motor_izquierdo(self, velocidad):
        self.motor_izquierdo.on(self.SpeedRadPS(velocidad))

    @property
    def dc_motor_izquierdo(self):
        return self.motor_izquierdo.duty_cycle

    @dc_motor_izquierdo.setter
    def dc_motor_izquierdo(self, ciclo):
        self.motor_izquierdo.run_direct(duty_cycle_sp = ciclo)

    @property
    def dc_motor_derecho(self):
        return self.motor_derecho.duty_cycle

    @dc_motor_derecho.setter
    def dc_motor_derecho(self, ciclo):
        self.motor_derecho.run_direct(duty_cycle_sp = ciclo)

    #Ambos motores
    def correr(self, linear, angular):
        derecha = ((linear)+((angular*self.sruedas)/2))/self.radio
        izquierda = ((linear)-((angular*self.sruedas)/2))/self.radio
        self.dos_motores.on(self.SpeedRadPS(izquierda), self.SpeedRadPS(derecha))

    def correr_tiempo(self, linear, angular, seconds, bloqueo):
        derecha = ((linear)+((angular*self.sruedas)/2))/self.radio
        izquierda = ((linear)-((angular*self.sruedas)/2))/self.radio
        self.dos_motores.on_for_seconds(self.SpeedRadPS(izquierda), self.SpeedRadPS(derecha), seconds, block = bloqueo)

    def parar(self):
        self.dos_motores.off()

    @property
    def velocidad_linear(self):
        return ((self.w_motor_derecho+self.w_motor_izquierdo)/2)*self.radio

    @property
    def velocidad_angular(self):
        return ((self.w_motor_derecho-self.w_motor_izquierdo)*self.radio)/self.sruedas


class localizacion(movimiento):
    def __init__(self, motor_izquierdo, motor_derecho, diametro_rueda, separacion_ruedas, posicion, modo):
        movimiento.__init__(self, motor_izquierdo, motor_derecho, diametro_rueda, separacion_ruedas)

        self.perimetro_rueda = 2*pi*self.radio

        self.posicion_robot = posicion

        #Odometria
        self.izquierda_anterior = self.motor_izquierdo.position
        self.derecha_anterior = self.motor_derecho.position
        self.tiempo_anterior = time()
        self.modo = modo

        #Probabilística
        self.margen_inicial_x_y = 0.05
        self.margen_inicial_angulo = 0.573
        self.mu =  np.array([[self.posicion_robot[0]],
                              [self.posicion_robot[1]],
                              [self.posicion_robot[3]]])
        self.sigma = np.array([[(self.margen_inicial_x_y/4)**2, 0.0, 0.0],
                                [0.0, (self.margen_inicial_x_y/4)**2, 0.0],
                                [0.0, 0.0, (self.margen_inicial_angulo/4)**2]])

        #Fichero
        self.f = None
        self.escribir_fichero_activo = False
        self.fin_escribir_fichero = True

    # def simular_sensor(self, posicion):
    #     mapa = [[0.0, 0.0], [0.0, 1.0], [1.0, 1.0], [1.0, 0.0]]

    #     return

    def odometria(self):
        izquierda_actual = self.motor_izquierdo.position
        derecha_actual = self.motor_derecho.position
        tiempo_actual = time()

        ticks_izquierda = izquierda_actual - self.izquierda_anterior
        ticks_derecha = derecha_actual - self.derecha_anterior
        h = tiempo_actual - self.tiempo_anterior

        if ticks_izquierda or ticks_derecha or h:
            self.izquierda_anterior = izquierda_actual
            self.derecha_anterior = derecha_actual
            self.tiempo_anterior = tiempo_actual

            rotacion_izquierda = float(ticks_izquierda / self.motor_izquierdo.count_per_rot)
            rotacion_derecha = float(ticks_derecha / self.motor_derecho.count_per_rot)

            distancia_izquierda = float(rotacion_izquierda * self.perimetro_rueda)
            distancia_derecha = float(rotacion_derecha * self.perimetro_rueda)

            distancia_total = (distancia_izquierda + distancia_derecha) / 2.0
            rotacion_total = (distancia_derecha - distancia_izquierda) / self.sruedas

            v = distancia_total / h

            if (self.modo == "euler"): #Euler
                self.posicion_robot[0] += distancia_total * cos(self.posicion_robot[3])
                self.posicion_robot[1] += distancia_total * sin(self.posicion_robot[3])
                self.posicion_robot[3] += rotacion_total

            elif (self.modo == "RK_2"): #Runge-Kutta de segundo orden
                self.posicion_robot[0] += distancia_total * cos(self.posicion_robot[3] + (rotacion_total/2))
                self.posicion_robot[1] += distancia_total * sin(self.posicion_robot[3] + (rotacion_total/2))
                self.posicion_robot[3] += rotacion_total

            elif (self.modo == "RK_4"): #Runge-Kutta de cuarto orden
                k01 = v * cos(self.posicion_robot[3])
                k02 = (v + 0.5*h) * cos(self.posicion_robot[3] + 0.5*k01*h)
                k03 = (v + 0.5*h) * cos(self.posicion_robot[3] + 0.5*k02*h)
                k04 = (v + h) * cos(self.posicion_robot[3] + k03*h)

                k11 = v * sin(self.posicion_robot[3])
                k12 = (v + 0.5*h) * sin(self.posicion_robot[3] + 0.5*k11*h)
                k13 = (v + 0.5*h) * sin(self.posicion_robot[3] + 0.5*k12*h)
                k14 = (v + h) * sin(self.posicion_robot[3] + k13*h)

                self.posicion_robot[0] += (1/6)*h*(k01 + 2*(k02 + k03) + k04)
                self.posicion_robot[1] += (1/6)*h*(k11 + 2*(k12 + k13) + k14)
                self.posicion_robot[3] += rotacion_total

        return self.posicion_robot

    def localizacion_probabilistica(self):
        mu_pred = [0.0, 0.0, 0.0]

        izquierda_actual = self.motor_izquierdo.position
        derecha_actual = self.motor_derecho.position

        ticks_izquierda = izquierda_actual - self.izquierda_anterior
        ticks_derecha = derecha_actual - self.derecha_anterior


        self.izquierda_anterior = izquierda_actual
        self.derecha_anterior = derecha_actual

        rotacion_izquierda = float(ticks_izquierda / self.motor_izquierdo.count_per_rot)
        rotacion_derecha = float(ticks_derecha / self.motor_derecho.count_per_rot)

        distancia_izquierda = float(rotacion_izquierda * self.perimetro_rueda)
        distancia_derecha = float(rotacion_derecha * self.perimetro_rueda)

        distancia_total = (distancia_izquierda + distancia_derecha) / 2.0
        rotacion_total = (distancia_derecha - distancia_izquierda) / self.sruedas

        mu_pred[0] = self.mu[0] + distancia_total * cos(self.mu[3])
        mu_pred[1] = self.mu[1] + distancia_total * sin(self.mu[3])
        mu_pred[3] = self.mu[3] + rotacion_total

        G = np.array([[1.0, 0.0, -distancia_total*sin(self.posicion_robot[3])],
                      [0.0, 1.0, distancia_total*cos(self.posicion_robot[3])],
                      [0.0, 0.0, 1.0]])

        Q = np.array([[(0.05*(mu_pred[0]-self.mu[0]))**2, 0.0, 0.0],
                      [0.0, (0.05*(mu_pred[1]-self.mu[1]))**2, 0.0],
                      [0.0, 0.0, (0.573*(mu_pred[3]-self.mu[3]))**2]])

        sigma_pred = G @ self.sigma @ G.T + Q

        #observacion_expectada = self.simular_sensor(mu_pred)

    def empezar_posicion_fichero(self, nombre_fichero, tiempo_espera):
        def hilo_fichero():
            i = 0
            while self.escribir_fichero_activo:
                self.f.write(str(i)+" "+str(self.odometria())+"\n")
                i = i + 1
                sleep(tiempo_espera)

            self.fin_escribir_fichero = True

        self.f = open(nombre_fichero,"w")
        self.escribir_fichero_activo = True
        self.fin_escribir_fichero = False
        self.id_hilo_fichero = Thread(target = hilo_fichero)
        self.id_hilo_fichero.start()

    def parar_posicion_fichero(self):
        self.escribir_fichero_activo = False
        if not self.fin_escribir_fichero:
            self.id_hilo_fichero.join(timeout=None)
        self.f.close()

class navegacion(localizacion):
    def __init__(self, motor_izquierdo, motor_derecho, diametro_rueda, separacion_ruedas, posicion, modo):
        localizacion.__init__(self, motor_izquierdo, motor_derecho, diametro_rueda, separacion_ruedas, posicion, modo)
        self.s = sensores_y_bateria(INPUT_1, INPUT_4)

    def coordenadas_global_a_robot(self, posicion_robot, punto_global):
        R = np.array([[cos(posicion_robot[3]), -sin(posicion_robot[3]), 0],
                      [sin(posicion_robot[3]), cos(posicion_robot[3]), 0],
                      [0.0, 0.0, 1.0]])

        Rt = R.T
        aux = -(Rt @ posicion_robot[:3])

        T = np.array([[Rt[0][0], Rt[0][1], Rt[0][2], aux[0]],
                     [Rt[1][0], Rt[1][1], Rt[1][2], aux[1]],
                     [Rt[2][0], Rt[2][1], Rt[2][2], aux[2]],
                     [0, 0, 0, 1]])

        resultado = T @ np.append(np.array(punto_global), 1)

        return resultado[:3].tolist()

    def navegacion_reactiva_campos_virtuales(self, punto_destino):
        vector_resultante = [0.0, 0.0, 0.0]
        KA = 1.0
        KR = 4.0

        while 1:
            vector_hasta_destino = self.coordenadas_global_a_robot(self.odometria(), punto_destino)
            modulo = np.sqrt(np.array(vector_hasta_destino) @ np.array(vector_hasta_destino))
            if (modulo <= 0.05):
                break

            if (self.s.distancia_sonar - 0.09) > 0.45:
                distancia_obstaculo = 0
            else:
                distancia_obstaculo = 0.45 - (self.s.distancia_sonar - 0.09)

            vector_resultante[0] = KA*vector_hasta_destino[0] - KR*distancia_obstaculo
            vector_resultante[1] = KA*vector_hasta_destino[1]
            vector_resultante[2] = KA*vector_hasta_destino[2]

            if((0.2 * vector_resultante[0]) > 0.5):
                v = 0.5
            else:
                v = 0.2 * vector_resultante[0]

            if((2 * vector_resultante[1]) > 2*pi):
                w = 2 * pi
            else:
                w = 2 * vector_resultante[1]

            self.correr(v, w)

            sleep(0.1)

        self.parar()

    def navegacion_planificada(self, puntos_objetivos):
        KW = 1.0
        vector_hasta_destino = [0, 0, 0]

        for punto in puntos_objetivos:
            while 1:
                posicion_robot = self.odometria()
                vector_hasta_destino[0] = punto[0] - posicion_robot[0]
                vector_hasta_destino[1] = punto[1] - posicion_robot[1]
                vector_hasta_destino[2] = punto[2] - posicion_robot[2]

                modulo = np.sqrt(np.array(vector_hasta_destino) @ np.array(vector_hasta_destino))

                if (modulo <= 0.05):
                    break

                angulo_objetivo = atan2(vector_hasta_destino[1], vector_hasta_destino[0])
                if angulo_objetivo < 0:
                    angulo_objetivo = angulo_objetivo + 2*pi

                angulo_robot = posicion_robot[3]
                while angulo_robot > 2*pi:
                    angulo_robot = angulo_robot - 2*pi
                while angulo_robot < 0:
                    angulo_robot = angulo_robot + 2*pi

                angulo = angulo_objetivo - angulo_robot
                if angulo < -pi:
                    angulo = angulo + 2*pi
                if angulo > pi:
                    angulo = -(2*pi - angulo)

                w = KW * angulo

                if w > 2*pi:
                    w = 2*pi
                if w < -2*pi:
                    w = -2*pi

                self.correr(0.2, w)

                sleep(0.1)
        self.parar()


