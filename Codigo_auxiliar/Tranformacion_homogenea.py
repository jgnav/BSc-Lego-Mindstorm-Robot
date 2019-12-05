import numpy as np
import math

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


#PRUEBA:
axisA = np.array([[1, 1, 0],
                  [0, -1, 0],
                  [0, 0, 1]])

axisB = np.array([[1, 2, 3],
                  [4, 5, 6],
                  [7, 8, 9]])

bPa0 = np.array([0, 0, 0])

aP = np.array([1, 1, 1])

bRa = rotation_matrix(axisA, axisB)

print(bRa)

bTa = inverted_translation_matrix(bPa0, bRa)

print(bTa)

bP = point_translation(aP, bPa0, axisA, axisB)

print(bP)
