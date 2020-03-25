import numpy as np
from math import cos, sin, pi, tan, atan2, sqrt

margen_inicial_x_y = 0.05
margen_inicial_angulo = 0.573
muk = np.array([0.300000000000000, 0.300000000000000, 1.570796326794897])
sigmak = np.array([[1.562500000000000e-04,0,0],[0,1.562500000000000e-04,0],[0,0,0.020520562500000]])

muk_pred = np.array([0.300000000000000,0.300000000000000,1.570796326794897])

G = np.array([[1,0,0],[0,1,0],[0,0,1]])

Q = np.array([[(0.05*(muk_pred[0]-muk[0]))**2, 0.0, 0.0],
                      [0.0, (0.05*(muk_pred[1]-muk[1]))**2, 0.0],
                      [0.0, 0.0, (0.573*(muk_pred[2]-muk[2]))**2]])

sigmak_pred = G @ sigmak @ G.T + Q

uTs =np.array([[-1.17700000000000, -0.0705000000000001, 0, 0.300000000000000], [-0.177000000000000,-1.07050000000000,0,0.300000000000000], [0,0,1,0],[-0.590000000000000,-0.235000000000000,0,1]])

calculationsok = 1
Hk = np.array([1.011244206193731,0,0.045621977356743])
mu_zk = 0.303373261858119

if calculationsok:
    Rk = 0.1**2
    sigma_zk = Hk @ sigmak_pred @ Hk.T + Rk

    sigmapok_pred = sigmak_pred @ Hk.T

    Kk = sigmapok_pred * (1/sigma_zk)

    muk = muk_pred + Kk * (0.3340 - mu_zk)
    sigmak = sigmak_pred - Kk @ (Hk @ sigmak_pred)

else:
        muk = muk_pred
        sigmak = sigmak_pred

print(muk)
print(sigmak)
