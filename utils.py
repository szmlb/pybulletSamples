# -*- coding: utf-8 -*
import numpy as np
import math
import sys

def getAngularVelocityFromMatrix(Rmat):
    """
    姿勢行列から各速度ベクトルを計算
    """

    Rmat_tmp = np.zeros((3, 3))
    Rmat_tmp[0,  0] = Rmat[0]
    Rmat_tmp[0,  1] = Rmat[1]
    Rmat_tmp[0,  2] = Rmat[2]

    Rmat_tmp[1,  0] = Rmat[3]
    Rmat_tmp[1,  1] = Rmat[4]
    Rmat_tmp[1,  2] = Rmat[5]

    Rmat_tmp[2,  0] = Rmat[6]
    Rmat_tmp[2,  1] = Rmat[7]
    Rmat_tmp[2,  2] = Rmat[8]

    theta = math.acos( (Rmat_tmp[0, 0] + Rmat_tmp[1, 1] + Rmat_tmp[2, 2] - 1.0) / 2.0 )

    omega = np.zeros((3, 1))
    if np.abs(theta) < sys.float_info.epsilon:
        omega = np.zeros((3, 1))
    else:
        omega[0, 0] = theta / (2.0 * math.sin(theta)) * (Rmat_tmp[2, 1] - Rmat_tmp[1, 2])
        omega[1, 0] = theta / (2.0 * math.sin(theta)) * (Rmat_tmp[0, 2] - Rmat_tmp[2, 0])
        omega[2, 0] = theta / (2.0 * math.sin(theta)) * (Rmat_tmp[1, 0] - Rmat_tmp[0, 1])

    return omega;
