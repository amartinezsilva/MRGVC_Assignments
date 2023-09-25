#####################################################################################
#
# MRGCV Unizar - Computer vision - Laboratory 1
#
# Title: 2D-3D geometry in homogeneous coordinates and camera projection
#
# Date: 14 September 2022
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

from line2DFittingSVD import drawLine



# Ensamble T matrix
def ensamble_T(R_w_c, t_w_c) -> np.array:
    """
    Ensamble the a SE(3) matrix with the rotation matrix and translation vector.
    """
    T_w_c = np.zeros((4, 4))
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

if __name__ == '__main__':
    np.set_printoptions(precision=4,linewidth=1024,suppress=True)


    # Load ground truth
    R_w_c1 = np.loadtxt('R_w_c1.txt')
    R_w_c2 = np.loadtxt('R_w_c2.txt')

    t_w_c1 = np.loadtxt('t_w_c1.txt')
    t_w_c2 = np.loadtxt('t_w_c2.txt')

    T_w_c1 = ensamble_T(R_w_c1, t_w_c1)
    T_w_c2 = ensamble_T(R_w_c2, t_w_c2)


    K_c = np.loadtxt('K.txt')

    X_A = np.array([3.44, 0.80, 0.82])
    A = [[3.44], [0.8], [0.82], [1]]
    X_B = np.array([4.20, 0.80, 0.82])
    B = [[4.2], [0.8], [0.82], [1]]
    X_C = np.array([4.2, 0.60, 0.82])
    C = [[4.2], [0.6], [0.82], [1]]
    X_D = np.array([3.55, 0.60, 0.82])
    D = [[3.55], [0.6], [0.82], [1]]
    X_E = np.array([-0.01, 2.6, 1.21])
    E = [[-0.01], [2.6], [1.21], [1]]

    print(np.array([[3.44, 0.80, 0.82]]).T) #transpose need to have dimension 2
    print(np.array([3.44, 0.80, 0.82]).T) #transpose does not work with 1 dim arrays

    # Example of transpose (need to have dimension 2)  and concatenation in numpy
    X_w = np.vstack((np.hstack((np.reshape(X_A,(3,1)), np.reshape(X_C,(3,1)))), np.ones((1, 2))))

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
    plotLabelled3DPoints(ax, X_w, ['A','C'], 'r', (-0.3, -0.3, 0.1)) # For plotting with labels (choose one of the both options)

    #Matplotlib does not correctly manage the axis('equal')
    xFakeBoundingBox = np.linspace(0, 4, 2)
    yFakeBoundingBox = np.linspace(0, 4, 2)
    zFakeBoundingBox = np.linspace(0, 4, 2)
    plt.plot(xFakeBoundingBox, yFakeBoundingBox, zFakeBoundingBox, 'w.')

    #Drawing a 3D segment
    draw3DLine(ax, X_A, X_C, '--', 'k', 1)

    print('Close the figure to continue. Left button for orbit, right button for zoom.')
    plt.show()

    ## 2D plotting example
    img1 = cv2.cvtColor(cv2.imread("Image1.jpg"), cv2.COLOR_BGR2RGB)
    img2 = cv2.cvtColor(cv2.imread("Image2.jpg"), cv2.COLOR_BGR2RGB)

    x1 = np.array([[527.7253, 441.7, 334.1983, 433.5, 1036.9],[292.9017, 393.8 ,392.1474, 303.7, 43.9]])

    plt.figure(1)
    plt.imshow(img1)
    plt.plot(x1[0, :], x1[1, :],'+r', markersize=15)
    plotLabeledImagePoints(x1, ['a','b','c','d','e'], 'r', (20,-20)) # For plotting with labels (choose one of the both options)
    plotNumberedImagePoints(x1, 'r', (20,25)) # For plotting with numbers (choose one of the both options)
    plt.title('Image 1')
    plt.draw()  # We update the figure display
    print('Click in the image to continue...')
    plt.waitforbuttonpress()

    # Exercise 1
    #print(K_c)
    #print(np.concatenate((np.identity(3),np.array([[0],[0],[0]])), axis=1))
    #print(T_w_c1)
    #print(np.linalg.inv(T_w_c1))
    RT1 = np.dot(np.concatenate((np.identity(3),np.array([[0],[0],[0]])), axis=1), np.linalg.inv(T_w_c1))
    P1 = np.dot(K_c, RT1) 
    RT2 = np.dot(np.concatenate((np.identity(3),np.array([[0],[0],[0]])), axis=1), np.linalg.inv(T_w_c2))
    P2 = np.dot(K_c, RT2)

    #print(P1)
    #print(P2)

    a1 = np.dot(P1,A)/2.7877
    a2 = np.dot(P2,A)/1.1287
    b1 = np.dot(P1,B)/2.0923
    b2 = np.dot(P2,B)/0.41
    c1 = np.dot(P1,C)/2.1138
    c2 = np.dot(P2,C)/0.3914
    d1 = np.dot(P1,D)/2.7085
    d2 = np.dot(P2,D)/1.0061
    e1 = np.dot(P1,E)/5.5996
    e2 = np.dot(P2,E)/4.437
    #print(a1)
    #print(b1)
    #print(c1)
    #print(d1)
    #print(e1)
    #print(a2)
    #print(b2)
    #print(c2)
    #print(d2)
    #print(e2)

    lab1 = np.cross(a1,b1, axis=0)
    lab1 = lab1.reshape((3,))
    drawLine(lab1, '-g', 1)
    #print(lab1)

    #lab2 = np.cross(a2,b2, axis=0)
    #lab2 = lab2.reshape((3,))
    #drawLine(lab2, '-g', 1)

    lcd1 = np.cross(c1,d1, axis=0)
    lcd1 = lcd1.reshape((3,))
    drawLine(lcd1, '-g', 1)

    #lcd2 = np.cross(c2,d2, axis=0)
    #lcd2 = lcd2.reshape((3,))
    #drawLine(lcd2, '-g', 1)

    plt.draw()
    print('Click in the image to continue...')
    plt.waitforbuttonpress()

    p_12 = np.cross(lab1,lcd1, axis=0) 
    p_12 = p_12/p_12[2]
    print()
    print("Point of intersection of the lines p_12 (equal to ab_inf):")
    print(p_12)

    point = np.array([[p_12[0]],[p_12[1]]])
    plt.plot(point[0], point[1],'+r', markersize=15)
    plotLabeledImagePoints(point, ['p_12'], 'r', (20,-20)) # For plotting with labels (choose one of the both options)
    plt.draw()
    print('Click in the image to continue...')
    plt.waitforbuttonpress()

    AB_inf = X_B - X_A
    AB_inf = np.array([AB_inf[0], AB_inf[1], AB_inf[2], 0])
    print()
    print("Line from A to B:")
    print(AB_inf)

    ab_inf = np.dot(P1,AB_inf)
    ab_inf = ab_inf/ab_inf[2]
    print()
    print("Line a to b in the image:")
    print(ab_inf)


    #3rd exersice
    #print(X_w)
    X_w = np.hstack((np.reshape(X_A,(3,1)), np.reshape(X_B,(3,1))))
    X_w = np.hstack((X_w, np.reshape(X_C,(3,1))))
    X_w = np.vstack((np.hstack((X_w, np.reshape(X_D,(3,1)))), np.ones((1, 4))))
    A = (X_w[:,0:4]).T
    print()
    print()
    print("The matrix A:")
    print(A)
    U, s, vh = np.linalg.svd(A)
    m_till = np.reshape(vh[-1, :], (4, 1))
    print()
    print("The plane result of SVD from A B C D")
    print(m_till)

    print()
    print("Distant between the plane and A")
    distanceA = (m_till[0]*X_A[0] + m_till[1]*X_A[1] + m_till[2]*X_A[2] + m_till[3]) / np.sqrt(m_till[0]**2 + m_till[1]**2 + m_till[2]**2)
    print(distanceA)

    print()
    print("Distant between the plane and B")
    distanceB = (m_till[0]*X_B[0] + m_till[1]*X_B[1] + m_till[2]*X_B[2] + m_till[3]) / np.sqrt(m_till[0]**2 + m_till[1]**2 + m_till[2]**2)
    print(distanceB)

    print()
    print("Distant between the plane and C")
    distanceC = (m_till[0]*X_C[0] + m_till[1]*X_C[1] + m_till[2]*X_C[2] + m_till[3]) / np.sqrt(m_till[0]**2 + m_till[1]**2 + m_till[2]**2)
    print(distanceC)

    print()
    print("Distant between the plane and D")
    distanceD = (m_till[0]*X_D[0] + m_till[1]*X_D[1] + m_till[2]*X_D[2] + m_till[3]) / np.sqrt(m_till[0]**2 + m_till[1]**2 + m_till[2]**2)
    print(distanceD)

    print()
    print("Distant between the plane and E")
    distanceE = (m_till[0]*X_E[0] + m_till[1]*X_E[1] + m_till[2]*X_E[2] + m_till[3]) / np.sqrt(m_till[0]**2 + m_till[1]**2 + m_till[2]**2)
    print(distanceE)

