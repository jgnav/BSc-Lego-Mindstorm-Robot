import numpy as np
from math import cos, sin, pi, tan, atan2, sqrt

def cutwithwall(xk, yk, pk, xp1, yp1, xp2, yp2):
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

    u = [xc-xk, yc-yk]
    r = [cos(pk), sin(pk)]
    if (u[0]*r[0]+u[1]*r[1]) < 0:
        thereis=0
        return thereis, xc, yc

    u = [xp2-xp1, yp2-yp1]
    c = [xc-xp1, yc-yp1]
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

def planeposesonar(Ts2u):
    originsonar = Ts2u @ np.array([0, 0, 0, 1])
    endingsonar = Ts2u @ np.array([1, 0, 0, 1])
    posesonar = [originsonar[0], originsonar[1], atan2(endingsonar[1]-originsonar[1], endingsonar[0]-originsonar[0])]
    return posesonar

def raycasting(mapa, poser, Ts2u):
    posesonar = planeposesonar(Ts2u)

    nps = len(mapa)
    cuts = []
    for f in range(0, nps):
        thereis0, xc0, yc0 = cutwithwall(posesonar[0], posesonar[1], posesonar[2], mapa[f][0], mapa[f][1], mapa[f][2], mapa[f][3])
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

def eq3(mapa, muk_pred, Ts2u):
    calculationsok = 0
    Hk = np.array([0.0, 0.0, 0.0])
    ok = 0

    indwall, xc, yc, mu_zk, drc = raycasting(mapa, muk_pred, Ts2u)
    if indwall == -1:
        return calculationsok, Hk, mu_zk

    xp1 = mapa[indwall][0]
    yp1 = mapa[indwall][1]
    xp2 = mapa[indwall][2]
    yp2 = mapa[indwall][3]

    posesonar = planeposesonar(Ts2u)
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
        Hk[0] = tanbeta/den
        Hk[1] = -1/den
        Hk[2] = -(-(posesonar[1]-yp1)+(posesonar[0]-xp1)*tanbeta)*(costheta+sentheta*tanbeta)/(den**2)
    else:
        cotbeta = cosbeta/senbeta
        den = costheta-sentheta*cotbeta
        Hk[0] = -1/den
        Hk[1] = cotbeta/den
        Hk[2] = -(-(posesonar[0]-xp1)+(posesonar[1]-yp1)*cotbeta)*(-sentheta+costheta*cotbeta)/(den**2)

    calculationsok = 1

    return calculationsok, Hk, mu_zk

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

mapa = [[0, 1.03, 0, 0], [1.03, 1.03, 0, 1.03], [1.03, 0, 1.03, 1], [0 ,0 ,1.03, 0]]

calculationsok, Hk, mu_zk = eq3(mapa, muk_pred, uTs)

if calculationsok:
    Rk = 0.2**2
    sigma_zk = Hk.dot(sigmak_pred)
    sigma_zk = sigma_zk.dot(Hk.transpose())
    sigma_zk = sigma_zk + Rk

    sigmapok_pred = sigmak_pred.dot(Hk.transpose())

    Kk = sigmapok_pred * (1/sigma_zk)

    muk = muk_pred + Kk * (0.03340 - mu_zk)

    sigmak = sigmak_pred - Kk.dot(Hk.dot(sigmak_pred))

else:
    muk = muk_pred
    sigmak = sigmak_pred

print(muk)
print(sigmak)
#print(Kk)
#print(Hk)
#print(mu_zk)
#print(sigma_zk)


