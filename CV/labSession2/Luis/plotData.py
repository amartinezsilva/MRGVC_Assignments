#####################################################################################
#
# MRGCV Unizar - Computer vision - Laboratory 2
#
# Title: Homography, Fundamental Matrix and Two View SfM
#
# Date: 16 September 2022
#
#####################################################################################
#
# Authors: Jesus Bermudez, Richard Elvira, Jose Lamarca, JMM Montiel
#
# Version: 1.0
#
#####################################################################################

from mpl_toolkits.mplot3d import Axes3D

import matplotlib.pyplot as plt
import numpy as np
import cv2

coords = []

def onclick(event):
    global ix, iy
    ix, iy = event.xdata, event.ydata

    global coords
    coords.append((ix, iy))

    return coords

def drawLine(l,strFormat,lWidth):
    """
    Draw a line
    -input:
      l: image line in homogenous coordinates
      strFormat: line format
      lWidth: line width
    -output: None
    """
    # p_l_y is the intersection of the line with the axis Y (x=0)
    print(-l[2] / l[1])
    p_l_y = np.array([0, -l[2] / l[1]])
    # p_l_x is the intersection point of the line with the axis X (y=0)
    p_l_x = np.array([-l[2] / l[0], 0])
    # Draw the line segment p_l_x to  p_l_y
    plt.plot([p_l_y[0], p_l_x[0]], [p_l_y[1], p_l_x[1]], strFormat, linewidth=lWidth)

# Ensamble T matrix
def ensamble_T(R_w_c, t_w_c) -> np.array:
    """
    Ensamble the a SE(3) matrix with the rotation matrix and translation vector.
    """
    T_w_c = np.zeros((4, 4), dtype=np.float32)
    T_w_c[0:3, 0:3] = R_w_c
    T_w_c[0:3, 3] = t_w_c
    T_w_c[3, 3] = 1
    return T_w_c


def plotLabeledImagePoints(x, labels, strColor,offset):
    """
        Plot indexes of points on a 2D image.
         -input:
             x: Points coordinates.
             strColor: Color of the text.
             offset: Offset from the point to the text.
         -output: None
         """
    for k in range(x.shape[1]):
        plt.text(x[0, k]+offset[0], x[1, k]+offset[1], labels[k], color=strColor)


def plotNumberedImagePoints(x,strColor,offset):
    """
        Plot indexes of points on a 2D image.
         -input:
             x: Points coordinates.
             strColor: Color of the text.
             offset: Offset from the point to the text.
         -output: None
         """
    for k in range(x.shape[1]):
        plt.text(x[0, k]+offset[0], x[1, k]+offset[1], str(k), color=strColor)


def plotLabelled3DPoints(ax, X, labels, strColor, offset):
    """
        Plot indexes of points on a 3D plot.
         -input:
             ax: axis handle
             X: Points coordinates.
             strColor: Color of the text.
             offset: Offset from the point to the text.
         -output: None
         """
    for k in range(X.shape[1]):
        ax.text(X[0, k]+offset[0], X[1, k]+offset[1], X[2,k]+offset[2], labels[k], color=strColor)

def plotNumbered3DPoints(ax, X,strColor, offset):
    """
        Plot indexes of points on a 3D plot.
         -input:
             ax: axis handle
             X: Points coordinates.
             strColor: Color of the text.
             offset: Offset from the point to the text.
         -output: None
         """
    for k in range(X.shape[1]):
        ax.text(X[0, k]+offset[0], X[1, k]+offset[1], X[2,k]+offset[2], str(k), color=strColor)

def draw3DLine(ax, xIni, xEnd, strStyle, lColor, lWidth):
    """
    Draw a segment in a 3D plot
    -input:
        ax: axis handle
        xIni: Initial 3D point.
        xEnd: Final 3D point.
        strStyle: Line style.
        lColor: Line color.
        lWidth: Line width.
    """
    ax.plot([np.squeeze(xIni[0]), np.squeeze(xEnd[0])], [np.squeeze(xIni[1]), np.squeeze(xEnd[1])], [np.squeeze(xIni[2]), np.squeeze(xEnd[2])],
            strStyle, color=lColor, linewidth=lWidth)

def drawRefSystem(ax, T_w_c, strStyle, nameStr):
    """
        Draw a reference system in a 3D plot: Red for X axis, Green for Y axis, and Blue for Z axis
    -input:
        ax: axis handle
        T_w_c: (4x4 matrix) Reference system C seen from W.
        strStyle: lines style.
        nameStr: Name of the reference system.
    """
    draw3DLine(ax, T_w_c[0:3, 3:4], T_w_c[0:3, 3:4] + T_w_c[0:3, 0:1], strStyle, 'r', 1)
    draw3DLine(ax, T_w_c[0:3, 3:4], T_w_c[0:3, 3:4] + T_w_c[0:3, 1:2], strStyle, 'g', 1)
    draw3DLine(ax, T_w_c[0:3, 3:4], T_w_c[0:3, 3:4] + T_w_c[0:3, 2:3], strStyle, 'b', 1)
    ax.text(np.squeeze( T_w_c[0, 3]+0.1), np.squeeze( T_w_c[1, 3]+0.1), np.squeeze( T_w_c[2, 3]+0.1), nameStr)

def triangulate3DPoints(x1, x2, T_w_c1, T_w_c2):
    RT1 = np.dot(np.concatenate((np.identity(3),np.array([[0],[0],[0]])), axis=1), np.linalg.inv(T_w_c1))
    P1 = np.dot(K_c, RT1) 
    RT2 = np.dot(np.concatenate((np.identity(3),np.array([[0],[0],[0]])), axis=1), np.linalg.inv(T_w_c2))
    P2 = np.dot(K_c, RT2)

    print()
    print("R1 and T1: ")
    print(RT1)

    print()
    print("R2 and T2: ")
    print(RT2)
    X_w_calculated = np.zeros((len(X_w[:][0]), len(X_w[0][:])))
    for i in range(len(x1[0][:])):

        # print()
        # print()
        # print("Point: ", i)
        A = np.zeros((4,4))
        A[0][0] = P1[2][0]*x1[0][i]-P1[0][0]
        A[0][1] = P1[2][1]*x1[0][i]-P1[0][1]
        A[0][2] = P1[2][2]*x1[0][i]-P1[0][2]
        A[0][3] = P1[2][3]*x1[0][i]-P1[0][3]

        A[1][0] = P1[2][0]*x1[1][i]-P1[1][0]
        A[1][1] = P1[2][1]*x1[1][i]-P1[1][1]
        A[1][2] = P1[2][2]*x1[1][i]-P1[1][2]
        A[1][3] = P1[2][3]*x1[1][i]-P1[1][3]

        A[2][0] = P2[2][0]*x2[0][i]-P2[0][0]
        A[2][1] = P2[2][1]*x2[0][i]-P2[0][1]
        A[2][2] = P2[2][2]*x2[0][i]-P2[0][2]
        A[2][3] = P2[2][3]*x2[0][i]-P2[0][3]

        A[3][0] = P2[2][0]*x2[1][i]-P2[1][0]
        A[3][1] = P2[2][1]*x2[1][i]-P2[1][1]
        A[3][2] = P2[2][2]*x2[1][i]-P2[1][2]
        A[3][3] = P2[2][3]*x2[1][i]-P2[1][3]

        U, s, vh = np.linalg.svd(A)
        m_till = np.reshape(vh[-1, :], (4, 1))
        m_till[0] = m_till[0]/m_till[3] 
        m_till[1] = m_till[1]/m_till[3] 
        m_till[2] = m_till[2]/m_till[3]
        m_till[3] = m_till[3]/m_till[3]
        X_w_calculated[0][i] = m_till[0]
        X_w_calculated[1][i] = m_till[1]
        X_w_calculated[2][i] = m_till[2]
        X_w_calculated[3][i] = m_till[3]

        # print()
        # print("The 3D result of SVD: ")
        # print(m_till)

        # print()
        # print("The 3D solution: ")
        # print(X_w[0][i])
        # print(X_w[1][i])
        # print(X_w[2][i])
    return X_w_calculated

def point_transfer(x1FloorData, x2FloorData, H_2_1):

    x2HomographyFloorData = np.dot(H_2_1, x1FloorData)
    x2HomographyFloorData = x2HomographyFloorData / x2HomographyFloorData[2][:]

    H_1_2 = np.linalg.inv(H_2_1)
    x1HomographyFloorData = np.dot(H_1_2, x2FloorData)
    x1HomographyFloorData = x1HomographyFloorData / x1HomographyFloorData[2][:]

    print("Points transfer from using Homography:")
    print(x2HomographyFloorData)

    fig = plt.figure(6)

    plt.imshow(img1, cmap='gray', vmin=0, vmax=255)
    plt.plot(x1FloorData[0, :], x1FloorData[1, :],'rx', markersize=10)
    plt.plot(x1HomographyFloorData[0, :], x1HomographyFloorData[1, :],'bx', markersize=10)

    plotNumberedImagePoints(x1FloorData, 'r', (10,0)) # For plotting with numbers (choose one of the both options)
    plotNumberedImagePoints(x1HomographyFloorData, 'b', (10,0)) # For plotting with numbers (choose one of the both options)

    plt.title('Image 1')
    plt.draw()  # We update the figure display
    print('Close the figure to continue. Left button for orbit, right button for zoom.')
    plt.show()

    fig = plt.figure(7)

    plt.imshow(img2, cmap='gray', vmin=0, vmax=255)
    plt.plot(x2HomographyFloorData[0, :], x2HomographyFloorData[1, :],'bx', markersize=10)
    plt.plot(x2FloorData[0, :], x2FloorData[1, :],'rx', markersize=10)

    plotNumberedImagePoints(x2FloorData, 'r', (10,0)) # For plotting with numbers (choose one of the both options)
    plotNumberedImagePoints(x2HomographyFloorData, 'b', (10,0)) # For plotting with numbers (choose one of the both options)

    plt.title('Image 2')
    plt.draw()  # We update the figure display

    print('Click in the image to continue...')
    plt.waitforbuttonpress()


if __name__ == '__main__':
    np.set_printoptions(precision=4,linewidth=1024,suppress=True)


    # Load ground truth
    T_w_c1 = np.loadtxt('T_w_c1.txt')
    T_w_c2 = np.loadtxt('T_w_c2.txt')

    K_c = np.loadtxt('K_c.txt')
    X_w = np.loadtxt('X_w.txt')

    x1 = np.loadtxt('x1Data.txt')
    x2 = np.loadtxt('x2Data.txt')


    ##Plot the 3D cameras and the 3D points
    fig3D = plt.figure(3)

    ax = plt.axes(projection='3d', adjustable='box')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    drawRefSystem(ax, np.eye(4, 4), '-', 'W')
    drawRefSystem(ax, T_w_c1, '-', 'C1')
    drawRefSystem(ax, T_w_c2, '-', 'C2')

    ax.scatter(X_w[0, :], X_w[1, :], X_w[2, :], marker='.')
    plotNumbered3DPoints(ax, X_w, 'r', (0.1, 0.1, 0.1)) # For plotting with numbers (choose one of the both options)

    #Matplotlib does not correctly manage the axis('equal')
    xFakeBoundingBox = np.linspace(0, 4, 2)
    yFakeBoundingBox = np.linspace(0, 4, 2)
    zFakeBoundingBox = np.linspace(0, 4, 2)
    plt.plot(xFakeBoundingBox, yFakeBoundingBox, zFakeBoundingBox, 'w.')
    print('Close the figure to continue. Left button for orbit, right button for zoom.')
    plt.show()

    ## 2D plotting example
    img1 = cv2.cvtColor(cv2.imread('image1.png'), cv2.COLOR_BGR2RGB)
    img2 = cv2.cvtColor(cv2.imread('image2.png'), cv2.COLOR_BGR2RGB)


    # plt.figure(1)
    # plt.imshow(img1, cmap='gray', vmin=0, vmax=255)
    # plt.plot(x1[0, :], x1[1, :],'rx', markersize=10)
    # plotNumberedImagePoints(x1, 'r', (10,0)) # For plotting with numbers (choose one of the both options)
    # plt.title('Image 1')
    # plt.draw()  # We update the figure display
    # print('Click in the image to continue...')
    # plt.waitforbuttonpress()

    # plt.figure(2)
    # plt.imshow(img2, cmap='gray', vmin=0, vmax=255)
    # plt.plot(x2[0, :], x2[1, :],'rx', markersize=10)
    # plotNumberedImagePoints(x2, 'r', (10,0)) # For plotting with numbers (choose one of the both options)
    # plt.title('Image 2')
    # plt.draw()  # We update the figure display
    # print('Click in the image to continue...')
    # plt.waitforbuttonpress()



    #######################################################
    #################### EXERCISE 1 #######################
    #######################################################

    X_w_calculated = triangulate3DPoints(x1, x2, T_w_c1, T_w_c2)

    ##Plot the 3D cameras and the 3D points
    
    # fig3D = plt.figure(5)

    # ax = plt.axes(projection='3d', adjustable='box')
    # ax.set_xlabel('X')
    # ax.set_ylabel('Y')
    # ax.set_zlabel('Z')

    # drawRefSystem(ax, np.eye(4, 4), '-', 'W')
    # drawRefSystem(ax, T_w_c1, '-', 'C1')
    # drawRefSystem(ax, T_w_c2, '-', 'C2')

    # ax.scatter(X_w_calculated[0, :], X_w_calculated[1, :], X_w_calculated[2, :], marker='.')
    # #plotNumbered3DPoints(ax, X_w_calculated, 'r', (0.1, 0.1, 0.1)) # For plotting with numbers (choose one of the both options)

    # #Matplotlib does not correctly manage the axis('equal')
    # xFakeBoundingBox = np.linspace(0, 4, 2)
    # yFakeBoundingBox = np.linspace(0, 4, 2)
    # zFakeBoundingBox = np.linspace(0, 4, 2)
    # plt.plot(xFakeBoundingBox, yFakeBoundingBox, zFakeBoundingBox, 'w.')
    # print('Close the figure to continue. Left button for orbit, right button for zoom.')
    # plt.show()
    

    #######################################################
    #################### EXERCISE 2 #######################
    #######################################################

    ## Section 2.1 

    # We draw the epipole line in the image 2
    F_21_test = np.loadtxt('F_21_test.txt')
    print()
    print("F test:")
    print(F_21_test)
    det_F_test = np.linalg.det(F_21_test)
    print("Determinant of F:", det_F_test)

    # We get the point from the image 1
    # fig5 = plt.figure(5)
    # plt.imshow(img1, cmap='gray', vmin=0, vmax=255)
    # plt.plot(x1[0, :], x1[1, :],'rx', markersize=10)
    # plotNumberedImagePoints(x1, 'r', (10,0)) # For plotting with numbers (choose one of the both options)
    # plt.title('Image 1')
    # plt.draw()  # We update the figure display
    # cid = fig5.canvas.mpl_connect('button_press_event', onclick)
    # print()
    # print('Click in the image to continue...')
    # plt.waitforbuttonpress()

    # print()
    # coord_image1 = [coords[0][0], coords[0][1], 1]
    # print(coord_image1)

    # epipole_test_line = np.dot(F_21_test, coord_image1)
    # print("Epipole test line: ")
    # print(epipole_test_line)
    # print()
    # fig6 = plt.figure(6)
    # plt.imshow(img2, cmap='gray', vmin=0, vmax=255)
    # plt.plot(x2[0, :], x2[1, :],'rx', markersize=10)
    # plotNumberedImagePoints(x2, 'r', (10,0)) # For plotting with numbers (choose one of the both options)
    # plt.title('Image 2')
    # drawLine(epipole_test_line, '-b', 1)

    # plt.draw()  # We update the figure display
    
    # print('Click in the image to continue...')
    # plt.waitforbuttonpress()

    
    ## Section 2.2
    
    # To calculate E we have to use the equation: E = [t]x R
    # In this way we need to calculate R_c2_c1 and [t] (which comes from t_c2_c1)
    T_c2_w = np.linalg.inv(T_w_c2)
    T_c2_c1 = np.dot(T_c2_w, T_w_c1)

    print()
    print("T c2 c1:")
    print(T_c2_c1)

    R_c2_c1 = T_c2_c1[0:3].T[0:3].T
    print()
    print("R c2 c1:")
    print(R_c2_c1)

    t_x = np.zeros((3,3)) 
    t_x[0][0] = 0
    t_x[0][1] = -T_c2_c1[2][3]
    t_x[0][2] = T_c2_c1[1][3]
    t_x[1][0] = T_c2_c1[2][3]
    t_x[1][1] = 0
    t_x[1][2] = -T_c2_c1[0][3]
    t_x[2][0] = -T_c2_c1[1][3]
    t_x[2][1] = T_c2_c1[0][3]
    t_x[2][2] = 0

    print()
    print("t_x:")
    print(t_x)

    # We obteain E by equations
    E = np.dot(t_x, R_c2_c1)
    print()
    print("E:")
    print(E)
    det_E = np.linalg.det(E)
    print("Determinant of E:", det_E)

    print("Essential Matrix E:")
    print(E)

    # We calculate K from E: F = KEK
    K_inverse_transpose = np.linalg.inv(K_c).T
    K_inverse = np.linalg.inv(K_c)
    F = np.dot(K_inverse_transpose, E)
    F = np.dot(F, K_inverse)
    # print()
    # print("F:")

    # print(F)
    # det_F = np.linalg.det(F)
    # print("Determinant of F:", det_F)

    # print("Epipole line: ")
    # epipole_line = np.dot(F, coord_image1)
    
    # drawLine(epipole_line, '-g', 1)
    # plt.draw()  # We update the figure display
    
    # print('Click in the image to continue...')
    # plt.waitforbuttonpress()


    ## Section 2.3

    # We obtain E by SVD
    # Construct the A matrix for the linear system
    A = np.zeros((len(x1[0][:]), 9))
    w_0 = 1
    w_1 = 1

    for i in range(len(x1[0][:])):
        A[i][0] = x1[0][i] * x2[0][i]
        A[i][1] = x1[0][i] * x2[1][i]
        A[i][2] = w_0 * x1[0][i]
        A[i][3] = x1[1][i] * x2[0][i]
        A[i][4] = x1[1][i] * x2[1][i]
        A[i][5] = w_0 * x1[1][i]
        A[i][6] = x2[0][i] * w_1
        A[i][7] = x2[1][i] * w_1
        A[i][8] = w_0 * w_1

    U, s, vh = np.linalg.svd(A)
    F_SVD = np.reshape(vh[-1, :], (3, 3))

    # print()
    # print("F with SVD:")
    # print(F_SVD)
    # det_F = np.linalg.det(F_SVD)
    # print("Determinant of F with SVD:", det_F)

    # print("Epipole line with SVD: ")
    # epipole_line_SVD = np.dot(F_SVD.T, coord_image1)
    
    # drawLine(epipole_line_SVD, '-y', 1)
    # plt.draw()  # We update the figure display
    
    # print('Click in the image to continue...')
    # plt.waitforbuttonpress()

 
    ## Section 2.4

    E_21 = np.dot(K_c.T, np.dot(F_21_test, K_c))

    # Assuming you have E_21 (3x3 Essential Matrix)
    U, S, Vt = np.linalg.svd(E_21)

    # Ensure that U and Vt are proper rotation matrices (det(U) = 1, det(Vt) = 1)
    # if np.linalg.det(U) < 0:
    #     U *= -1
    # if np.linalg.det(Vt) < 0:
    #     Vt *= -1

    W = np.array([[0, -1, 0],
                [1, 0, 0],
                [0, 0, 1]])

    # Possible R and t combinations
    Rplus90_positive = np.dot(U, np.dot(W, Vt.T))
    Rplus90_negative = - np.dot(U, np.dot(W, Vt.T))
    Rminus90_positive = np.dot(U, np.dot(W.T, Vt.T))
    Rminus90_negative = - np.dot(U, np.dot(W.T, Vt.T))
    t1 = U[:, 2]
    t2 = -U[:, 2]

    print()
    print("Determinant of Rplus90_positive")
    print(np.linalg.det(Rplus90_positive)) #det= -1
    print("Determinant of Rplus90_negative")
    print(np.linalg.det(Rplus90_negative)) #det= 1
    print("Determinant of Rminus90_positive")
    print(np.linalg.det(Rminus90_positive)) #det= -1
    print("Determinant of Rminus90_negative")
    print(np.linalg.det(Rminus90_negative)) #det= 1

    R1 = Rplus90_negative
    R2 = Rminus90_negative
    # # Four possible combinations of R and t
    poses = [(R1, t1), (R1, t2), (R2, t1), (R2, t2)]

    for idx, (R, t) in enumerate(poses):
        T_c2_to_c1 = np.eye(4)
        T_c2_to_c1[:3, :3] = R
        T_c2_to_c1[:3, 3] = t
        T_c2_to_c1[3, 3] = 1
        T_w_to_c1 = np.dot(T_w_c2, T_c2_to_c1)
        print("R and t solution")
        print(T_w_to_c1)

        X_world = triangulate3DPoints(x1, x2, T_w_to_c1, T_w_c2)
        # Apply the filter to discard points with values greater than 4
        X_world_filtered = X_world[:, np.all(np.abs(X_world) <= 4, axis=0)]

        # Now, visualize the results for this solution
        # fig3D = plt.figure(idx+6)

        # ax = plt.axes(projection='3d', adjustable='box')
        # ax.set_xlabel('X')
        # ax.set_ylabel('Y')
        # ax.set_zlabel('Z')

        # drawRefSystem(ax, np.eye(4, 4), '-', 'W')
        # drawRefSystem(ax, T_w_to_c1, '-', 'C1')
        # drawRefSystem(ax, T_w_c2, '-', 'C2')

        # ax.scatter(X_world_filtered[0, :], X_world_filtered[1, :], X_world_filtered[2, :], marker='.')
        # ax.scatter(X_w[0, :], X_w[1, :], X_w[2, :], color='orange', marker='.')
        # #plotNumbered3DPoints(ax, X_w_calculated, 'r', (0.1, 0.1, 0.1)) # For plotting with numbers (choose one of the both options)

        # #Matplotlib does not correctly manage the axis('equal')
        # xFakeBoundingBox = np.linspace(0, 4, 2)
        # yFakeBoundingBox = np.linspace(0, 4, 2)
        # zFakeBoundingBox = np.linspace(0, 4, 2)
        # plt.plot(xFakeBoundingBox, yFakeBoundingBox, zFakeBoundingBox, 'w.')
        # plt.title(f'Solution {idx+1}')
        # print('Close the figure to continue. Left button for orbit, right button for zoom.')
        # plt.show()

    ############
    # We choose R2 and t1, the third option

    ## Section 3.1

    # Given pose
    Pi_1 = np.array([[0.0149,0.9483,0.3171,-1.7257]]).T 
    print(Pi_1)

    # Given plane coefficients
    a = Pi_1[0][0]
    b = Pi_1[1][0]
    c = Pi_1[2][0]
    d = Pi_1[3][0]

    Plane_normal = np.array([[a,b,c]]).T

    T_c2_c1 = np.dot(T_c2_w, T_w_c1)
    t_c2_c1 = np.array(T_c2_c1[-1][0:3])
    print(t_c2_c1.shape)
    print(Plane_normal.shape)

    Matrix = R_c2_c1 - np.dot(t_c2_c1, Plane_normal) / d

    # Calculate the homography matrix
    H = np.dot(K_c, np.dot(Matrix, np.linalg.inv(K_c)))

    # Normalize homography matrix (optional)
    #H /= H[2, 2]

    print()
    print("Homography:")
    print(H)


    ## Section 3.2

    x1FloorData = np.loadtxt("x1FloorData.txt")
    x2FloorData = np.loadtxt("x2FloorData.txt")
    point_transfer(x1FloorData, x2FloorData, H)

    ## Section 3.3

    n_matches = x1FloorData.shape[1]
    print("Floor point matches:")
    print(n_matches)

    A = np.zeros((n_matches*2, 9))
    j = 0
    for i in range(n_matches):

        x_0 = x1FloorData[0][i]
        x_1 = x2FloorData[0][i]
        y_0 = x1FloorData[1][i]
        y_1 = x2FloorData[1][i]

        A[j][0] = x_0
        A[j][1] = y_0
        A[j][2] = 1.0
        A[j][3] = 0.0
        A[j][4] = 0.0
        A[j][5] = 0.0
        A[j][6] = -x_1*x_0
        A[j][7] = -x_1*y_0
        A[j][8] = -x_1

        A[j+1][0] = 0
        A[j+1][1] = 0
        A[j+1][2] = 0
        A[j+1][3] = x_0
        A[j+1][4] = y_0
        A[j+1][5] = 1.0
        A[j+1][6] = -y_1*x_0
        A[j+1][7] = -y_1*y_0
        A[j+1][8] = -y_1

        j = j+2

    print(A.shape)
    u, s, vh = np.linalg.svd(A)
    H_matches_vector = np.reshape(vh[-1, :],(9,1))

    H_21_matches = np.zeros((3,3))
    H_21_matches[0][0] = H_matches_vector[0]
    H_21_matches[0][1] = H_matches_vector[1]
    H_21_matches[0][2] = H_matches_vector[2]
    H_21_matches[1][0] = H_matches_vector[3]
    H_21_matches[1][1] = H_matches_vector[4]
    H_21_matches[1][2] = H_matches_vector[5]
    H_21_matches[2][0] = H_matches_vector[6]
    H_21_matches[2][1] = H_matches_vector[7]
    H_21_matches[2][2] = H_matches_vector[8]

    print("H21 from matches:")
    print(H_21_matches)

    point_transfer(x1FloorData, x2FloorData, H_21_matches)



    
