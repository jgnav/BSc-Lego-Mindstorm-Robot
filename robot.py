#!/usr/bin/env python3

from ev3dev2.motor import Motor, SpeedRPS, MoveTank
from ev3dev2.sensor.lego import ColorSensor, UltrasonicSensor
from ev3dev2.power import PowerSupply
from math import pi, sin, cos, atan, atan2, tan, sqrt
from threading import Thread
import numpy as np
from time import sleep, time
from ev3dev2.sensor import INPUT_1, INPUT_4
from ev3dev2.stopwatch import StopWatch

class sensores_y_bateria:
    def __init__(self, sonar, sensor_color):
        self.sonar = UltrasonicSensor(sonar)
        self.s_color = ColorSensor(sensor_color)
        self.bateria = PowerSupply()

    #Sensor ultrasónico
    @property
    def distancia_sonar(self):
        return (self.sonar.distance_centimeters / 100)

    @property
    def otros_sensores_presentes(self):
        return self.sonar.other_sensor_present

    #Sensor color
    def calibrar_blanco(self):
        self.s_color.calibrate_white()

    @property
    def color(self):
        return self.s_color.color

    @property
    def nombre_color(self):
        return self.s_color.color_name

    @property
    def ambiente(self):
        return self.s_color.ambient_light_intensity

    @property
    def reflexion(self):
        return self.s_color.reflected_light_intensity

    @property
    def rgb(self):
        return self.s_color.rgb

    #Batería
    @property
    def voltaje_bateria(self):
        return self.bateria.measured_volts

    @property
    def corriente_bateria(self):
        return self.bateria.measured_amps

class motores:
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
    
    @property
    def posicion_motor_derecho(self):
        return self.motor_derecho.position * pi / 180 

    @property
    def posicion_motor_izquierdo(self):
        return self.motor_izquierdo.position * pi / 180

    #Ambos motores
    def correr(self, linear, angular):
        derecha = ((linear)+((angular*self.sruedas)/2))/self.radio
        izquierda = ((linear)-((angular*self.sruedas)/2))/self.radio
        self.dos_motores.on(self.SpeedRadPS(izquierda), self.SpeedRadPS(derecha))

    def correr_tiempo(self, linear, angular, tiempo, bloqueo):
        derecha = ((linear)+((angular*self.sruedas)/2))/self.radio
        izquierda = ((linear)-((angular*self.sruedas)/2))/self.radio
        self.dos_motores.on_for_seconds(self.SpeedRadPS(izquierda), self.SpeedRadPS(derecha), tiempo, block = bloqueo)

    def parar(self):
        self.dos_motores.off()

    @property
    def velocidad_linear(self):
        return ((self.w_motor_derecho+self.w_motor_izquierdo)/2)*self.radio

    @property
    def velocidad_angular(self):
        return ((self.w_motor_derecho-self.w_motor_izquierdo)*self.radio)/self.sruedas


class localizacion(motores):
    def __init__(self, motor_izquierdo, motor_derecho, diametro_rueda, separacion_ruedas, posicion, loc_sonar, loc_sensor_color):
        motores.__init__(self, motor_izquierdo, motor_derecho, diametro_rueda, separacion_ruedas)

        self.s = sensores_y_bateria(sonar = loc_sonar, sensor_color = loc_sensor_color)
        self.perimetro_rueda = 2*pi*self.radio
        self.posicion_robot = posicion

        #Odometria
        self.izquierda_anterior = self.posicion_motor_izquierdo
        self.derecha_anterior = self.posicion_motor_derecho
        self.tiempo_anterior = time()

        #Probabilística
        self.margen_inicial_x_y = 0.05
        self.margen_inicial_angulo = 0.573
        self.muk = np.array([[self.posicion_robot[0][0]],
                             [self.posicion_robot[1][0]],
                             [self.posicion_robot[3][0]]])
        self.sigmak = np.array([[(self.margen_inicial_x_y/4)**2, 0.0, 0.0],
                                [0.0, (self.margen_inicial_x_y/4)**2, 0.0],
                                [0.0, 0.0, (self.margen_inicial_angulo/4)**2]])

        #Fichero
        self.f = None
        self.escribir_fichero_activo = False
        self.fin_escribir_fichero = True
        self.t = StopWatch()

    def cutwithwall(self, xk, yk, pk, xp1, yp1, xp2, yp2):
        xc = 0
        yc = 0
        dyp = yp2-yp1
        dxp = xp2-xp1
        denxc = dyp-dxp*tan(pk)
        if denxc == 0:
            thereis=0
            return thereis, xc, yc

        num = dyp*(xk-xp1)+dxp*(yp1-yk)
        xc = xk-num/denxc
        denyc = dxp-dyp*(1/tan(pk))
        if denyc == 0:
            thereis=0
            return thereis, xc, yc

        yc=yk+num/denyc

        u = np.array([xc-xk, yc-yk])
        r = np.array([cos(pk), sin(pk)])
        if (u[0]*r[0]+u[1]*r[1]) < 0:
            thereis=0
            return thereis, xc, yc

        u = np.array([xp2-xp1, yp2-yp1])
        c = np.array([xc-xp1, yc-yp1])
        mu = sqrt((u[0]**2)+(u[1]**2))
        mc = sqrt((c[0]**2)+(c[1]**2))
        if (u[0]*c[0]+u[1]*c[1]) < 0:
            thereis=0
            return thereis, xc, yc

        if mc > mu:
            thereis=0
            return thereis, xc, yc

        thereis=1

        return thereis, xc, yc

    def planeposesonar(self, Ts2u):
        originsonar = Ts2u @ np.array([[0], [0], [0], [1]])
        endingsonar = Ts2u @ np.array([[1], [0], [0], [1]])
        posesonar = [originsonar[0][0], originsonar[1][0], atan2(endingsonar[1][0]-originsonar[1][0], endingsonar[0][0]-originsonar[0][0])]
        return posesonar

    def raycasting(self, mapa, poser, Ts2u):
        posesonar = self.planeposesonar(Ts2u)

        nps = len(mapa)
        cuts = []
        for f in range(0, nps):
            thereis0, xc0, yc0 = self.cutwithwall(posesonar[0], posesonar[1], posesonar[2], mapa[f][0], mapa[f][1], mapa[f][2], mapa[f][3])
            if thereis0 == 1:
                d0 = sqrt(((xc0-posesonar[0])**2)+((yc0-posesonar[1])**2))
                cuts.append([f, xc0, yc0, d0])

        if cuts == []:
            indwall = -1
            xc = 0
            yc = 0
            dc = 0
            drc = 0
            return indwall, xc, yc, dc, drc

        aux = [row[3] for row in cuts]

        minc = min(aux)
        iminc = aux.index(minc)
        indwall = cuts[iminc][0]
        xc = cuts[iminc][1]
        yc = cuts[iminc][2]
        dc = cuts[iminc][3]
        drc = sqrt(((xc-poser[0])**2)+(yc-poser[1])**2)

        return indwall, xc, yc, dc, drc

    def eq3(self, mapa, muk_pred, Ts2u):
        calculationsok = 0
        Hk = np.array([[0.0, 0.0, 0.0]])
        ok = 0

        indwall, xc, yc, mu_zk, drc = self.raycasting(mapa, muk_pred, Ts2u)
        if indwall == -1:
            return calculationsok, Hk, mu_zk

        xp1 = mapa[indwall][0]
        yp1 = mapa[indwall][1]
        xp2 = mapa[indwall][2]
        yp2 = mapa[indwall][3]

        posesonar = self.planeposesonar(Ts2u)
        senbeta = yp2-yp1
        cosbeta = xp2-xp1
        sentheta = sin(posesonar[2])
        costheta = cos(posesonar[2])

        if ((senbeta == 0) and (sentheta == 0)) or ((cosbeta == 0) and (costheta == 0)):
            return calculationsok, Hk, mu_zk

        if (cosbeta != 0) and (costheta != 0):
            if (senbeta/cosbeta == sentheta/costheta):
                return calculationsok, Hk, mu_zk

        if (cosbeta != 0):
            tanbeta = senbeta/cosbeta
            den = sentheta-costheta*tanbeta
            Hk[0][0] = tanbeta/den
            Hk[0][1] = -1/den
            Hk[0][2] = -(-(posesonar[1]-yp1)+(posesonar[0]-xp1)*tanbeta)*(costheta+sentheta*tanbeta)/(den**2)
        else:
            cotbeta = cosbeta/senbeta
            den = costheta-sentheta*cotbeta
            Hk[0][0] = -1/den
            Hk[0][1] = cotbeta/den
            Hk[0][2] = -(-(posesonar[0]-xp1)+(posesonar[1]-yp1)*cotbeta)*(-sentheta+costheta*cotbeta)/(den**2)

        calculationsok = 1

        return calculationsok, Hk, mu_zk

    def coordenadas_robot_a_global(self, posicion):
        T = np.array([[cos(posicion[2][0]), -sin(posicion[2][0]), 0.0, posicion[0][0]],
                      [sin(posicion[2][0]), cos(posicion[2][0]), 0.0, posicion[1][0]],
                      [0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 1.0]])
        return T

    def odometria(self, modo):
        izquierda_actual = self.posicion_motor_izquierdo
        derecha_actual = self.posicion_motor_derecho
        tiempo_actual = time()

        ticks_izquierda = izquierda_actual - self.izquierda_anterior
        ticks_derecha = derecha_actual - self.derecha_anterior
        h = tiempo_actual - self.tiempo_anterior

        if (ticks_izquierda or ticks_derecha) and h:
            self.izquierda_anterior = izquierda_actual
            self.derecha_anterior = derecha_actual
            self.tiempo_anterior = tiempo_actual

            rotacion_izquierda = float(ticks_izquierda / self.motor_izquierdo.count_per_rot)
            rotacion_derecha = float(ticks_derecha / self.motor_derecho.count_per_rot)

            distancia_izquierda = float(rotacion_izquierda * self.perimetro_rueda)
            distancia_derecha = float(rotacion_derecha * self.perimetro_rueda)

            distancia_total = (distancia_izquierda + distancia_derecha) / 2.0
            rotacion_total = (distancia_derecha - distancia_izquierda) / self.sruedas

            if (modo == "euler"): #Euler
                self.posicion_robot[0] += distancia_total * cos(self.posicion_robot[3][0])
                self.posicion_robot[1] += distancia_total * sin(self.posicion_robot[3][0])
                self.posicion_robot[3] += rotacion_total

                return self.posicion_robot

            elif (modo == "RK2"): #Runge-Kutta de segundo orden
                self.posicion_robot[0] += distancia_total * cos(self.posicion_robot[3][0] + (rotacion_total / 2))
                self.posicion_robot[1] += distancia_total * sin(self.posicion_robot[3][0] + (rotacion_total / 2))
                self.posicion_robot[3] += rotacion_total

                return self.posicion_robot

            elif (modo == "RK4"): #Runge-Kutta de cuarto orden

                theta05 = self.posicion_robot[3][0] + (rotacion_total / 2)
                theta1 = self.posicion_robot[3][0] + rotacion_total
                self.posicion_robot[0] += distancia_total * (1/6) * (cos(self.posicion_robot[3][0]) + 4 * cos(theta05) + cos(theta1))
                self.posicion_robot[1] += distancia_total * (1/6) * (sin(self.posicion_robot[3][0]) + 4 * sin(theta05) + sin(theta1))
                self.posicion_robot[3] += rotacion_total

                return self.posicion_robot

            elif (modo == "Exacto"): #Exacto
                v = distancia_total / h
                w = rotacion_total / h
                theta1 = self.posicion_robot[3] + rotacion_total
                if w:    
                    self.posicion_robot[0] += (v / w) * (sin(theta1) - sin(self.posicion_robot[3]))
                    self.posicion_robot[1] -= (v / w) * (cos(theta1) - cos(self.posicion_robot[3]))
                    self.posicion_robot[3] += rotacion_total
                else:
                    self.posicion_robot[0] += (distancia_total * cos(self.posicion_robot[3]))
                    self.posicion_robot[1] += (distancia_total * sin(self.posicion_robot[3]))
                    self.posicion_robot[3] += rotacion_total

                return self.posicion_robot

            elif (modo == "Prob"): #Uso en localización probabilistica
                muk_pred = np.array([[self.muk[0][0] + (distancia_total * cos(self.muk[2][0]))],
                                     [self.muk[1][0] + (distancia_total * sin(self.muk[2][0]))],
                                     [self.muk[2][0] + rotacion_total]])

                G = np.array([[1.0, 0.0, -distancia_total * sin(self.muk[2][0])],
                              [0.0, 1.0, distancia_total * cos(self.muk[2][0])],
                              [0.0, 0.0, 1.0]])

                self.posicion_robot[0] += distancia_total * cos(self.posicion_robot[3][0]) 
                self.posicion_robot[1] += distancia_total * sin(self.posicion_robot[3][0])
                self.posicion_robot[3] += rotacion_total

                return muk_pred, G

    def localizacion_probabilistica(self, mapa, rox, roy, rotheta, Rk):
        muk_pred, G = self.odometria("Prob")

        Q = np.array([[(rox*(muk_pred[0][0]-self.muk[0][0]))**2, 0.0, 0.0],
                      [0.0, (roy*(muk_pred[1][0]-self.muk[1][0]))**2, 0.0],
                      [0.0, 0.0, (rotheta*(muk_pred[2][0]-self.muk[2][0]))**2]])

        sigmak_pred = G @ self.sigmak @ G.T + Q

        offsx = -0.059
        offsy = -0.0235
        offsphi = pi/2
        rTs = np.array([[cos(offsphi), -sin(offsphi), 0, offsx],
                        [sin(offsphi), cos(offsphi), 0, offsy],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
        uTr = self.coordenadas_robot_a_global(muk_pred)
        uTs = uTr @ rTs

        calculationsok, Hk, mu_zk = self.eq3(mapa, muk_pred, uTs)
        distancia = self.s.distancia_sonar

        if calculationsok:
            sigma_zk = Hk[0] @ sigmak_pred @ Hk.T + Rk

            sigmapok_pred = sigmak_pred @ Hk.T

            Kk = sigmapok_pred * (1/sigma_zk)

            self.muk = muk_pred + Kk * (distancia - mu_zk)
            self.sigmak = sigmak_pred - Kk @ (Hk @ sigmak_pred)

        else:
            self.muk = muk_pred
            self.sigmak = sigmak_pred

        return self.muk, self.sigmak

    def empezar_posicion_fichero(self, nombre_fichero):
        def hilo_fichero():
            self.t.start()
            while self.escribir_fichero_activo:
                self.f.write(str(self.t.value_ms)+" "+str(self.posicion_motor_izquierdo)+" "+str(self.posicion_motor_derecho)+"\n")

                """
                mapa = np.array([[0, 1.03, 0, 0],
                                 [1.03, 1.03, 0, 1.03],
                                 [1.03, 0, 1.03, 1],
                                 [0 ,0 ,1.03, 0]])
                rox = 0.05
                roy = 0.05
                rotheta = 0.573
                Rk = 0.1**2
                mu, sigma, distancia = self.localizacion_probabilistica(mapa, rox, roy, rotheta, Rk)
                self.f.write(str(mu[0][0])+" "+str(mu[1][0])+" "+str(mu[2][0])+" "+str(sigma[0][0])+" "+str(sigma[0][1])+" "+str(sigma[0][2])+" "+str(sigma[1][0])+" "+str(sigma[1][1])+" "+str(sigma[1][2])+" "+str(sigma[2][0])+" "+str(sigma[2][1])+" "+str(sigma[2][2])+" "+str(self.posicion_robot[0][0])+" "+str(self.posicion_robot[1][0])+" "+str(self.posicion_robot[2][0])+" "+str(self.posicion_robot[3][0])+" "+str(distancia)+"\n")
                """
            self.t.stop()
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
    def __init__(self, motor_izquierdo, motor_derecho, diametro_rueda, separacion_ruedas, posicion, nav_sonar, nav_sensor_color):
        localizacion.__init__(self, motor_izquierdo, motor_derecho, diametro_rueda, separacion_ruedas, posicion, nav_sonar, nav_sensor_color)
 
    def coordenadas_global_a_robot(self, posicion_robot, punto_global):
        R = np.array([[cos(posicion_robot[3][0]), -sin(posicion_robot[3][0]), 0],
                      [sin(posicion_robot[3][0]), cos(posicion_robot[3][0]), 0],
                      [0.0, 0.0, 1.0]])

        Rt = R.transpose()
        aux = -(Rt @ posicion_robot[:3])

        T = np.array([[Rt[0][0], Rt[0][1], Rt[0][2], aux[0][0]],
                     [Rt[1][0], Rt[1][1], Rt[1][2], aux[1][0]],
                     [Rt[2][0], Rt[2][1], Rt[2][2], aux[2][0]],
                     [0, 0, 0, 1]])

        p = np.array([[punto_global[0][0]], [punto_global[1][0]], [punto_global[2][0]], 1])
        resultado = T @ p

        return resultado[:3]
   

    def navegacion_reactiva_campos_virtuales(self, objetivo, KA, KR):
        vector_resultante = np.array([0.0, 0.0, 0.0])

        while 1:
            vector_hasta_destino = self.coordenadas_global_a_robot(self.odometria("RK_2"), objetivo)

            modulo = sqrt(vector_hasta_destino[0].T @ vector_hasta_destino[0])
            if (modulo <= 0.05):
                break

            if (self.s.distancia_sonar - 0.09) > 0.45:
                distancia_obstaculo = 0
            else:
                distancia_obstaculo = 0.45 - (self.s.distancia_sonar - 0.09)

            vector_resultante[0] = KA * vector_hasta_destino[0][0] - KR * distancia_obstaculo
            vector_resultante[1] = KA * vector_hasta_destino[1][0]
            vector_resultante[2] = KA * vector_hasta_destino[2][0]

            v = 0.2 * vector_resultante[0]
            w = 2 * vector_resultante[1]

            if(v > 0.2):
                v = 0.2
            if(v < -0.2):
                v = -0.2

            if(w > pi):
                w = pi
            if(w < -pi):
                w = -pi

            self.correr(v, w)

        self.parar()

    def navegacion_planificada(self, puntos_objetivos):
        KW = 1.0
        vector_hasta_destino = np.array([0, 0, 0])

        puntos_objetivos = puntos_objetivos.transpose()

        for punto in puntos_objetivos:
            while 1:
                posicion_robot = self.odometria("RK2")
                vector_hasta_destino[0] = punto[0] - posicion_robot[0][0]
                vector_hasta_destino[1] = punto[1] - posicion_robot[1][0]
                vector_hasta_destino[2] = punto[2] - posicion_robot[2][0]

                modulo = sqrt(vector_hasta_destino @ vector_hasta_destino)
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

                if w > pi:
                    w = pi
                if w < -pi:
                    w = -pi

                self.correr(0.2, w)
        self.parar()


