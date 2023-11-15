#####################################################################################
#
# MRGCV Unizar - Computer vision - Laboratory 3
#
# Title: Line fitting with least squares optimization
#
# Date: 26 October 2020
#
#####################################################################################
#
# Authors: Jesus Bermudez, Richard Elvira, Jose Lamarca, JMM Montiel
#
# Version: 1.0
#
#####################################################################################

import matplotlib.pyplot as plt
import numpy as np
import scipy
import scipy.optimize as scOptim


def normalize_array(x_unnormalized) -> np.array:
    x_normalized = x_unnormalized
    for i in range(len(x_normalized)):
        x_normalized[i] = x_unnormalized[i]/x_unnormalized[i][2]
    return x_normalized

def ensamble_T(R_w_c, t_w_c) -> np.array:
    """
    Ensamble the a SE(3) matrix with the rotation matrix and translation vector.
    """
    T_w_c = np.zeros((4, 4))
    T_w_c[0:3, 0:3] = R_w_c
    T_w_c[0:3, 3] = t_w_c
    T_w_c[3, 3] = 1
    return T_w_c

def resBundleProjection(Op, x1Data, x2Data, K_c, nPoints):     
    """     
    -input:         
    Op: Optimization parameters: this must include a paramtrization for T_21 (reference 1 seen from reference 2)             
    in a proper way and for X1 (3D points in ref 1)         
    x1Data: (3xnPoints) 2D points on image 1 (homogeneous coordinates)         
    x2Data: (3xnPoints) 2D points on image 2 (homogeneous coordinates) 
    K_c: (3x3) Intrinsic calibration matrix         
    nPoints: Number of points     
    -output:         
    res: residuals from the error between the 2D matched points and the projected points from the 3D points               
    (2 equations/residuals per 2D point)             
    """ 
    theta = Op[0:3]
    azimuth = Op[3]
    elevation = Op[4]
    X_w = Op[5:]

    res = []

    # calculating points in 2D from 3d to camera 1
    T_c1_w = np.eye(4)
    P1 = np.dot(np.concatenate((np.identity(3), np.array([[0],[0],[0]])), axis=1), T_c1_w)
    P1 = np.dot(K_c, P1)
        
    points_c1_unnormalized = np.dot(P1,X_w)

    points_c1 = normalize_array(points_c1_unnormalized.T).T

    # calculating points in 2D from 3d to camera 2
    R_2_1 = scipy.linalg.expm(crossMatrix(theta))
    t_2_1 = np.array([[np.sin(elevation)*np.cos(azimuth)], [np.sin(elevation)*np.sin(azimuth)], [np.cos(elevation)]]).reshape((3,1))
    T_2_1 = ensamble_T(R_2_1, t_2_1)

    P2 = np.dot(np.concatenate((np.identity(3), np.array([[0],[0],[0]])), axis=1), T_2_1)
    P2 = np.dot(K_c, P2)

    points_c2_unnormalized = np.dot(P2,X_w)

    points_c2 = normalize_array(points_c2_unnormalized.T).T

    # we get errors
    e1 = x1Data[0:2] - points_c1[0:2]
    e2 = x2Data[0:2] - points_c2[0:2]
    
    res = e1.flatten().tolist() + e2.flatten().tolist()
   
    return res


def crossMatrixInv(M):     
  x = [M[2, 1], M[0, 2], M[1, 0]]     
  return x  

def crossMatrix(x):     
  M = np.array([[0, -x[2], x[1]],                   
                [x[2], 0, -x[0]],                   
                [-x[1], x[0], 0]], dtype="object")     
  return M 


if __name__ == '__main__':
    
    T_w_c1 = np.loadtxt('T_w_c1.txt')
    T_w_c2 = np.loadtxt('T_w_c2.txt')
    T_w_c3 = np.loadtxt('T_w_c3.txt')

    K_c = np.loadtxt('K_c.txt')
    F_21 = np.loadtxt('F_21.txt')

    X_w = np.loadtxt('X_w.txt')
    x1Data = np.loadtxt('x1Data.txt')
    x2Data = np.loadtxt('x2Data.txt')
    x3Data = np.loadtxt('x3Data.txt')

    T_c2_w = np.linalg.inv(T_w_c2)
    T_c2_c1 = np.dot(T_c2_w,T_w_c1)
    print("T_c2_c1:")
    print(T_c2_c1)

    R_c2_c1 = T_c2_c1[0:3].T[0:3].T
    print("R_c2_c1")
    print(R_c2_c1)

    t_c2_c1 = T_c2_c1.T[-1][0:3]
    print("t_c2_c1:")
    print(t_c2_c1)

    # calculating points in 2D from 3d to camera 1
    T_c1_w = np.linalg.inv(T_w_c1)
    P1 = np.dot(np.concatenate((np.identity(3), np.array([[0],[0],[0]])), axis=1), T_c1_w)
    P1 = np.dot(K_c, P1)
        
    points_c1_unnormalized = np.dot(P1,X_w)

    points_c1 = normalize_array(points_c1_unnormalized.T).T

    # calculating points in 2D from 3d to camera 2
    T_c2_c1 = ensamble_T(R_c2_c1, t_c2_c1)
    T_c2_c1 = np.linalg.inv(T_c2_c1)

    X_w_c2 = X_w * T_c2_c1

    T_c2_w = np.linalg.inv(T_w_c2)
    P2 = np.dot(np.concatenate((np.identity(3), np.array([[0],[0],[0]])), axis=1), T_c2_w)
    P2 = np.dot(K_c, P2)

    points_c2_unnormalized = np.dot(P2,X_w_c2)

    points_c2 = normalize_array(points_c2_unnormalized.T).T




    theta = crossMatrixInv(scipy.linalg.logm(R_c2_c1)) 
    
    # theory 6.8
    elevation = np.arccos(t_c2_c1[2][0])
    azimuth = np.arcsin(t_c2_c1[0][0] / np.sin(elevation))

    Op = theta+[azimuth, elevation] + X_w[:3].flatten().tolist()

    print(X_w[1].shape)
    nPoints = X_w[1].shape
    res = resBundleProjection(Op, x1Data, x2Data, K_c, nPoints)

    OpOptim = scOptim.least_squares(resBundleProjection, Op, args=(x1Data, x2Data, K_c, nPoints,), method='lm')