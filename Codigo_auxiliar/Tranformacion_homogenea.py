import numpy as np
from math import cos, sin, pi


def _theta_a_ejes(theta):
    ejes = [[1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]]

    ejes[0][0] = cos(theta - (pi/2))
    ejes[1][0] = sin(theta - (pi/2))
    ejes[0][1] = cos(theta)
    ejes[1][1] = sin(theta)

    return ejes

def _matriz_de_rotacion(ejesA, ejesB):
    axisA = np.array(ejesA)
    axisB = np.array(ejesB)
    bRa = np.array([[(axisA[:, 0] @ axisB[:, 0]), (axisA[:, 1] @ axisB[:, 0]), (axisA[:, 2] @ axisB[:, 0])],
                    [(axisA[:, 0] @ axisB[:, 1]), (axisA[:, 1] @ axisB[:, 1]), (axisA[:, 2] @ axisB[:, 1])],
                    [(axisA[:, 0] @ axisB[:, 2]), (axisA[:, 1] @ axisB[:, 2]), (axisA[:, 2] @ axisB[:, 2])]])
    return bRa.tolist()


def _matriz_de_translacion(cero, m_rotacion):
    bPa0 = np.array(cero)
    bRa = np.array(m_rotacion)
    bTa = np.array([[bRa[0][0], bRa[0][1], bRa[0][2], bPa0[0]],
                    [bRa[1][0], bRa[1][1], bRa[1][2], bPa0[1]],
                    [bRa[2][0], bRa[2][1], bRa[2][2], bPa0[2]],
                    [0, 0, 0, 1]])
    return bTa.tolist()

def _matriz_de_translacion_inversa(cero, m_rotacion):
    bPa0 = np.array(cero)
    bRa = np.array(m_rotacion)
    bRaT = np.transpose(bRa)
    aux = -(bRaT @ bPa0)
    aTb = np.array([[bRaT[0][0], bRaT[0][1], bRaT[0][2], aux[0]],
                    [bRaT[1][0], bRaT[1][1], bRaT[1][2], aux[1]],
                    [bRaT[2][0], bRaT[2][1], bRaT[2][2], aux[2]],
                    [0, 0, 0, 1]])
    return aTb.tolist()

def _translacion_de_punto(aP, aPb0, axisA, axisB):
    bRa = _matriz_de_rotacion(axisA, axisB)
    aTb = _matriz_de_translacion_inversa(aPb0, bRa)
    aPprima = np.append(np.array(aP), 1)
    bP = np.array(aTb) @ aPprima
    return bP[:3].tolist()

robot = [1.0, 1.0, 1.0, pi]
aPb0 = robot[:3]
axisB = _theta_a_ejes(robot[3])
aPb0 = robot[:3]

axisA = [[1.0, 0.0, 0.0],
         [0.0, 1.0, 0.0],
         [0.0, 0.0, 1.0]]
aP = [1.0, 1.0, 1.0]

print(_translacion_de_punto(aP, aPb0, axisA, axisB))


"""
def rotation_matrix(axisA, axisB):
    bRa = np.array([[(axisA[0] @ axisB[0]), (axisA[1] @ axisB[0]), (axisA[2] @ axisB[0])],
                    [(axisA[0] @ axisB[1]), (axisA[1] @ axisB[1]), (axisA[2] @ axisB[1])],
                    [(axisA[0] @ axisB[2]), (axisA[1] @ axisB[2]), (axisA[2] @ axisB[2])]])
    return bRa

def translation_matrix(bPa0, bRa):
    bTa = np.array([[bRa[0][0], bRa[1][0], bRa[2][0], bPa0[0]],
                    [bRa[0][1], bRa[1][1], bRa[2][1], bPa0[1]],
                    [bRa[0][2], bRa[1][2], bRa[2][2], bPa0[2]],
                    [0, 0, 0, 1]])
    return bTa

def inverted_translation_matrix(bPa0, bRa):
    bRaT = np.transpose(bRa)
    aux = -(bRaT @ bPa0)

    aTb = np.array([[bRaT[0][0], bRaT[1][0], bRaT[2][0], aux[0]],
                    [bRaT[0][1], bRaT[1][1], bRaT[2][1], aux[1]],
                    [bRaT[0][2], bRaT[1][2], bRaT[2][2], aux[2]],
                    [0, 0, 0, 1]])
    return aTb

def point_translation(aP, bPa0, axisA, axisB):
    bRa = rotation_matrix(axisA, axisB)
    bTa = translation_matrix(bPa0, bRa)
    aPprima = np.append(aP, 1)
    bP = bTa @ aPprima
    return bP[:3]

# PRUEBA:
axisA = np.array([[1, 0, 0],
                  [0, 1, 0],
                  [0, 0, 1]])

axisB = np.array([[1, 0, 0],
                  [0, 1, 0],
                  [0, 0, 1]])

bPa0 = np.array([1, 1, 1])

aP = np.array([1, 1, 1])

bRa = rotation_matrix(axisA, axisB)

# print(bRa)

bTa = inverted_translation_matrix(bPa0, bRa)

# print(bTa)

bP = point_translation(aP, bPa0, axisA, axisB)

print(bP)
"""
"""
theta = ejes_a_theta(axisA)
axis = theta_a_ejes(theta)
print(axis)

self._cero_universal = np.array([0.0, 0.0, 0.0])
        self._ejes_universales = np.array([[1.0, 0.0, 0.0],
                                           [0.0, 1.0, 0.0],
                                           [0.0, 0.0, 1.0]])

self._cero_robot = np.array([0.0, 0.0, 0.0])
self._theta_robot = 0.0
self._ejes_robot = np.array([[1.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0],
                             [0.0, 0.0, 1.0]])
 """
"""
    def _ejes_a_theta(self, ejes):
        if ((ejes[0][1] > 0) and (ejes[1][1] >= 0)):

            theta = atan(ejes[1][1]/ejes[0][1])

        elif ((ejes[0][1] < 0) and (ejes[1][1] > 0)):

            theta = atan(ejes[1][1]/ejes[0][1]) + pi

        elif ((ejes[0][1] < 0) and (ejes[1][1] <= 0)):

            theta = atan(ejes[1][1]/ejes[0][1]) + pi

        elif ((ejes[0][1] > 0) and (ejes[1][1] < 0)):

            theta = 2*pi + atan(ejes[1][1]/ejes[0][1])

        elif ((ejes[0][1] == 0) and (ejes[1][1] > 0)):

            theta = pi/2

        elif ((ejes[0][1] == 0) and (ejes[1][1] < 0)):

            theta = (3/2)*pi

        return theta

    def _theta_a_ejes(self, theta):
        ejes = np.array([[1.0, 0.0, 0.0],
                         [0.0, 1.0, 0.0],
                         [0.0, 0.0, 1.0]])

        if ((theta > 0) and (theta < pi/2)):

            ejes[0][0] = 1
            ejes[0][1] = 1
            ejes[1][0] = 1
            ejes[1][1] = 1

        elif ((theta > pi/2) and (theta < pi)):

            ejes[0][0] = 1
            ejes[0][1] = 1
            ejes[1][0] = 1
            ejes[1][1] = 1

        elif ((theta > pi) and (theta < (3/2)*pi)):

            ejes[0][0] = 1
            ejes[0][1] = 1
            ejes[1][0] = 1
            ejes[1][1] = 1

        elif ((theta > (3/2)*pi) and (theta < 2*pi)):

            ejes[0][0] = 1
            ejes[0][1] = 1
            ejes[1][0] = 1
            ejes[1][1] = 1

        return ejes
"""
