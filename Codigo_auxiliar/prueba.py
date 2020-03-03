import numpy as np
from math import cos, sin, pi

posicion_inicial = [0.0, 0.0, 0.0, pi/2]

mu = posicion_inicial
sigma = np.array([[1, 2, 3],
                  [4, 5, 6],
                  [7, 8, 9]])

print(sigma.T)
