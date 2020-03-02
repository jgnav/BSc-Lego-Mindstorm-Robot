import numpy as np
from math import cos, sin, pi

posicion_inicial = [0.0, 0.0, 0.0, pi/2]

mu = posicion_inicial
sigma = np.array([[np.var(posicion_inicial[0]), 0.0, 0.0],
                         [0.0, np.var(posicion_inicial[1]), 0.0],
                         [0.0, 0.0, np.var(posicion_inicial[3])]])

print(sigma)
