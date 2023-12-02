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
import cv2


def kannala_projection(K_c, X, D):
    
    [k1,k2,k3,k4] = D[0:4]
    phi = np.arctan2(X[1,:],X[0,:])
    R = np.sqrt(pow(X[0,:],2) + pow(X[1,:],2))
    theta = np.arctan2(R, X[2,:])
    d = theta + k1*pow(theta,3) + k2 * pow(theta, 5) + k3 * pow(theta, 7) + k4 * pow(theta, 9)
    u = np.dot(K_c, np.array([d*np.cos(phi), d*np.sin(phi), [1]]))
    return u

def kannala_unprojection(K_c, u, D):

    [k1,k2,k3,k4] = D[0:4]
    point_c = np.dot(np.linalg.inv(K_c), u)
    phi = np.arctan2(point_c[1,:], point_c[0,:])
    num = pow(point_c[0,:], 2) + pow(point_c[1,:], 2)
    den = pow(point_c[2,:], 2)
    r = np.sqrt(num / den)
    theta = np.roots([k4, 0, k3, 0, k2, 0, k1, 0, 1, -r])
    theta = theta[np.isreal(theta)]

    v = np.array([[np.sin(theta) * np.cos(phi)], [np.sin(theta) * np.sin(phi)], [np.cos(phi)]]).reshape(3,1)

    return np.real(v)

def calculate_planes_from_v(v):

    vx = float(v[0,0])
    vy = float(v[1,0])
    vz = float(v[2,0])

    pi_sym = np.array([[-vy], [vx], [0], [0]]).reshape(4,1)
    pi_perp = np.array([[-vz*vx], [-vz*vy], [pow(vx,2) + pow(vy, 2)], [0]]).reshape(4,1)

    return pi_sym, pi_perp

if __name__ == '__main__':

    T_leftright = np.loadtxt('T_leftright.txt')
    T_wAwB_gt = np.loadtxt('T_wAwB_gt.txt')
    T_wAwB_seed = np.loadtxt('T_wAwB_seed.txt')
    T_wc1 = np.loadtxt('T_wc1.txt')
    T_wc2 = np.loadtxt('T_wc2.txt')

    K_1 = np.loadtxt('K_1.txt')
    K_2 = np.loadtxt('K_2.txt')

    x1Data = np.loadtxt('x1.txt')
    x2Data = np.loadtxt('x2.txt')
    x3Data = np.loadtxt('x3.txt')
    x3Data = np.loadtxt('x4.txt')

    fisheye1_frameA = cv2.cvtColor(cv2.imread('fisheye1_frameA.png'), cv2.COLOR_BGR2RGB)
    fisheye1_frameB = cv2.cvtColor(cv2.imread('fisheye1_frameB.png'), cv2.COLOR_BGR2RGB)
    fisheye2_frameA = cv2.cvtColor(cv2.imread('fisheye2_frameA.png'), cv2.COLOR_BGR2RGB)
    fisheye2_frameB = cv2.cvtColor(cv2.imread('fisheye2_frameB.png'), cv2.COLOR_BGR2RGB)

    D1_k_array = np.loadtxt('D1_k_array.txt')   
    D2_k_array = np.loadtxt('D2_k_array.txt')

    X1 = np.array([[3], [2], [10], [1]]).reshape(4,1)
    X2 = np.array([[-5], [6], [7], [1]]).reshape(4,1)
    X3 = np.array([[1], [5], [14], [1]]).reshape(4,1)

    u1 = np.array([[503.387], [450.1594], [1]]).reshape(3,1)
    u2 = np.array([[267.9465], [580.4671], [1]]).reshape(3,1)
    u3 = np.array([[441.0609], [493.0671], [1]]).reshape(3,1)
        
    #3.1
    #Kannala-Brandt projection model

    print("Projection with C1:")

    u1_c1 = kannala_projection(K_1, X1, D1_k_array)
    print(u1_c1)
    u2_c1 = kannala_projection(K_1, X2, D1_k_array)
    print(u2_c1)
    u3_c1 = kannala_projection(K_1, X3, D1_k_array)
    print(u3_c1)

    print("Projection with C2:")
    u1_c2 = kannala_projection(K_2, X1, D2_k_array)
    print(u1_c2)
    u2_c2 = kannala_projection(K_2, X2, D2_k_array)
    print(u2_c2)
    u3_c2 = kannala_projection(K_2, X3, D2_k_array)
    print(u3_c2)


    #Kannala-Brandt unprojection model

    print("Unprojection with C1:")

    v1_c1 = kannala_unprojection(K_1, u1, D1_k_array)
    v2_c1 = kannala_unprojection(K_1, u2, D1_k_array)
    v3_c1 = kannala_unprojection(K_1, u3, D1_k_array)

    print("Unprojection with C2:")
    v1_c2 = kannala_unprojection(K_2, u1, D2_k_array)
    v2_c2 = kannala_unprojection(K_2, u2, D2_k_array)
    v3_c2 = kannala_unprojection(K_2, u3, D2_k_array)
    
    ##3.2 Triangulation for pose A
    
    print("Exercise 3.2")
    ##Use for loop to get all v for every u, v point, thren translate to world and unproject to check

    npoints = x1Data.shape[1]
    X_triangulated = np.zeros((4, npoints))
    for i in range(npoints):

        u1_c1 = x1Data[:,i].reshape(3,1)
        u1_c2 = x2Data[:,i].reshape(3,1)

        v1_c1 = kannala_unprojection(K_1, u1_c1, D1_k_array)
        v1_c2 = kannala_unprojection(K_2, u1_c2, D2_k_array)
        
        #Triangulation
        pi_sym1, pi_perp1 = calculate_planes_from_v(v1_c1)
        pi_sym2, pi_perp2 = calculate_planes_from_v(v1_c2)

        pi_sym1_c2 = np.dot(T_leftright.T, pi_sym1)
        pi_perp1_c2 = np.dot(T_leftright.T, pi_perp1)

        A = np.zeros((4,4))
        A[0][:] = pi_sym1_c2.T
        A[1][:] = pi_perp1_c2.T
        A[2][:] = pi_sym2.T
        A[3][:] = pi_perp2.T

        print("Triangulation matrix A:")
        print(A)
        u, s, vh = np.linalg.svd(A)
        X = np.reshape(vh[-1, :],(4,1))

        X = X / X[3][:] 

        X_triangulated[0][i] = X[0,:]
        X_triangulated[1][i] = X[1,:]
        X_triangulated[2][i] = X[2,:]
        X_triangulated[3][i] = X[3,:]


    print(X_triangulated)

    #Check with projection model with one point

    i = 10
    X1 = X_triangulated[:,i].reshape(4,1)
    u1_c1 = kannala_projection(K_1, X1, D1_k_array)
    print("Projection triangulated first point in C1")
    print(u1_c1)
    print("Projection triangulated first point in C2")
    u1_c2 = kannala_projection(K_2, X1, D2_k_array)
    print(u1_c2)


    #Check with projection model for all points

    u_c1 = np.zeros((3,npoints))
    u_c2 = np.zeros((3,npoints))

    for i in range(npoints):
        X = X_triangulated[:,i].reshape(4,1)
        u_c1_p = kannala_projection(K_1, X, D1_k_array)

        u_c1[0][i] = u_c1_p[0,:]
        u_c1[1][i] = u_c1_p[1,:]
        u_c1[2][i] = u_c1_p[2,:]

        u_c2_p = kannala_projection(K_2, X, D2_k_array)

        u_c2[0][i] = u_c2_p[0,:]
        u_c2[1][i] = u_c2_p[1,:]
        u_c2[2][i] = u_c2_p[2,:]
    