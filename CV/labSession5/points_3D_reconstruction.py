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


def visualize_2D_points(image, real_points, estimated_points):
    """
    Visualize real and estimated 2D points on an image.

    Parameters:
    - image: The image to display.
    - real_points: A 2xN array representing the real 2D points.
    - estimated_points: A 2xN array representing the estimated 2D points.
    """

    # Create a figure and axis
    fig, ax = plt.subplots()

    # Display the image
    ax.imshow(image)

    # Plot real points with red arrows
    ax.quiver(real_points[0], real_points[1], np.zeros_like(real_points[0]), np.zeros_like(real_points[1]),
              angles='xy', scale_units='xy', scale=1, color='red', label='Real Points')

    # Plot estimated points with blue arrows
    ax.quiver(estimated_points[0], estimated_points[1], np.zeros_like(estimated_points[0]), np.zeros_like(estimated_points[1]),
              angles='xy', scale_units='xy', scale=1, color='blue', label='Estimated Points')

    # Plot lines connecting corresponding points
    for real_point, estimated_point in zip(real_points.T, estimated_points.T):
        ax.plot([real_point[0], estimated_point[0]], [real_point[1], estimated_point[1]], 'black', linestyle='dashed')

    # Set axis limits
    ax.set_xlim([0, image.shape[1]])
    ax.set_ylim([image.shape[0], 0])  # y-axis is inverted in images

    # Add legend
    ax.legend()

    # Show the plot
    plt.show()



def kannala_projection(K_c, X, D):
    
    [k1,k2,k3,k4] = D[0:4]
    phi = np.arctan2(X[1,:],X[0,:])
    R = np.sqrt(pow(X[0,:],2) + pow(X[1,:],2))
    theta = np.arctan2(R, X[2,:])
    d = theta + k1*pow(theta,3) + k2 * pow(theta, 5) + k3 * pow(theta, 7) + k4 * pow(theta, 9)
    u = K_c @ np.array([d*np.cos(phi), d*np.sin(phi), [1]])
    return u

def kannala_unprojection(K_c, u, D):

    [k1,k2,k3,k4] = D[0:4]
    point_c = np.linalg.inv(K_c) @ u
    phi = np.arctan2(point_c[1,0], point_c[0,0])
    num = pow(point_c[0,0], 2) + pow(point_c[1,0], 2)
    den = pow(point_c[2,0], 2)
    r = np.sqrt(num / den)
    theta = np.roots([k4, 0, k3, 0, k2, 0, k1, 0, 1, -r])
    theta = float(np.real(theta[np.isreal(theta)]))

    v = np.array([[np.sin(theta) * np.cos(phi)], [np.sin(theta) * np.sin(phi)], [np.cos(phi)]]).reshape(3,1)

    return v

def calculate_planes_from_v(v):

    vx = v[0,0]
    vy = v[1,0]
    vz = v[2,0]

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

    v1_c1 = kannala_unprojection(K_1, u1_c1, D1_k_array)
    print(v1_c1)
    v2_c1 = kannala_unprojection(K_1, u2_c1, D1_k_array)
    v3_c1 = kannala_unprojection(K_1, u3_c1, D1_k_array)

    print("Unprojection with C2:")
    v1_c2 = kannala_unprojection(K_2, u1_c2, D2_k_array)
    print(v1_c2)
    v2_c2 = kannala_unprojection(K_2, u2_c2, D2_k_array)
    v3_c2 = kannala_unprojection(K_2, u3_c2, D2_k_array)
    
    ##3.2 Triangulation for pose A
    
    print("Exercise 3.2")
    ##Use for loop to get all v for every u, v point, thren translate to world and unproject to check

    npoints = x1Data.shape[1]
    X_triangulated_c2 = np.zeros((4, npoints))
    for i in range(npoints):

        u1 = x1Data[:,i].reshape(3,1)
        u2 = x2Data[:,i].reshape(3,1)   

        v1 = kannala_unprojection(K_1, u1, D1_k_array)
        v2 = kannala_unprojection(K_2, u2, D2_k_array)
        
        #Triangulation
        pi_sym1, pi_perp1 = calculate_planes_from_v(v1)
        pi_sym2, pi_perp2 = calculate_planes_from_v(v2)

        pi_sym1_c2 = np.dot(T_leftright.T, pi_sym1)
        pi_perp1_c2 = np.dot(T_leftright.T, pi_perp1)

        A = np.zeros((4,4))
        A[0,:] = pi_sym1_c2.T
        A[1,:] = pi_perp1_c2.T
        A[2,:] = pi_sym2.T
        A[3,:] = pi_perp2.T

        u, s, vh = np.linalg.svd(A)
        X = np.reshape(vh[-1, :],(4,1))

        X = X / X[3][:] 

        X_triangulated_c2[0][i] = X[0,:]
        X_triangulated_c2[1][i] = X[1,:]
        X_triangulated_c2[2][i] = X[2,:]
        X_triangulated_c2[3][i] = X[3,:]


    X_triangulated_w = np.dot(T_wc2, X_triangulated_c2)
    X_triangulated_c1 = np.dot(T_leftright, X_triangulated_c2)
    #X_triangulated_c1 = np.dot(np.linalg.inv(T_wc1), X_triangulated_w)

    #Check with projection model for all points

    x1Data_tri = np.zeros((3,npoints))
    x2Data_tri = np.zeros((3,npoints))

    for i in range(npoints):

        X_c1 = X_triangulated_c1[:,i].reshape(4,1)
        X_c2 = X_triangulated_c2[:,i].reshape(4,1)
        X_w = X_triangulated_w[:,i].reshape(4,1)

        u_1 = kannala_projection(K_1, X_c1, D1_k_array)

        x1Data_tri[0,i] = u_1[0,:]
        x1Data_tri[1,i] = u_1[1,:]
        x1Data_tri[2,i] = u_1[2,:]

        u_2 = kannala_projection(K_2, X_c2, D2_k_array)

        x2Data_tri[0,i] = u_2[0,:]
        x2Data_tri[1,i] = u_2[1,:]
        x2Data_tri[2,i] = u_2[2,:]

    
    np.savetxt('x1Data_tri.txt', x1Data_tri)
    np.savetxt('x2Data_tri.txt', x2Data_tri)

    visualize_2D_points(fisheye1_frameA, x1Data, x1Data_tri)
    visualize_2D_points(fisheye2_frameA, x2Data, x2Data_tri)