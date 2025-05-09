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
import scipy.linalg as scAlg



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

def normalize_array(x_unnormalized) -> np.array:
    x_normalized = x_unnormalized
    for i in range(len(x_normalized)):
        x_normalized[i] = x_unnormalized[i]/x_unnormalized[i][2]
    return x_normalized

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
    X_B = np.array([4.20, 0.80, 0.82])
    X_C = np.array([4.20, 0.60, 0.82])
    X_D = np.array([3.55, 0.60, 0.82])
    X_E = np.array([-0.01, 2.60, 1.21])

    print(np.array([[3.44, 0.80, 0.82]]).T) #transpose need to have dimension 2
    print(np.array([3.44, 0.80, 0.82]).T) #transpose does not work with 1 dim arrays

    # Example of transpose (need to have dimension 2)  and concatenation in numpy
    X_w = np.vstack((np.hstack((np.reshape(X_A,(3,1)), np.reshape(X_B,(3,1)), np.reshape(X_C,(3,1)), np.reshape(X_D,(3,1)), np.reshape(X_E,(3,1)))), np.ones((1, 5))))

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
    plotLabelled3DPoints(ax, X_w, ['A','B','C','D','E'], 'r', (-0.3, -0.3, 0.1)) # For plotting with labels (choose one of the both options)

    #Matplotlib does not correctly manage the axis('equal')
    xFakeBoundingBox = np.linspace(0, 4, 2)
    yFakeBoundingBox = np.linspace(0, 4, 2)
    zFakeBoundingBox = np.linspace(0, 4, 2)
    plt.plot(xFakeBoundingBox, yFakeBoundingBox, zFakeBoundingBox, 'w.')

    #Drawing a 3D segment
    draw3DLine(ax, X_A, X_B, '--', 'k', 1)
    draw3DLine(ax, X_C, X_D, '--', 'k', 1)

    print('Close the figure to continue. Left button for orbit, right button for zoom.')
    plt.show()

    ## 2D plotting example
    img1 = cv2.cvtColor(cv2.imread("Image1.jpg"), cv2.COLOR_BGR2RGB)
    img2 = cv2.cvtColor(cv2.imread("Image2.jpg"), cv2.COLOR_BGR2RGB)

    #Exercise 1
    
    T_c1_w = np.linalg.inv(T_w_c1)
    P1 = np.dot(np.concatenate((np.identity(3), np.array([[0],[0],[0]])), axis=1), T_c1_w)
    P1 = np.dot(K_c, P1)
    print("P1 =")
    print(P1)

    T_c2_w = np.linalg.inv(T_w_c2)
    P2 = np.dot(np.concatenate((np.identity(3), np.array([[0],[0],[0]])), axis=1), T_c2_w)
    P2 = np.dot(K_c, P2)
    print("P2 =")
    print(P2)
        
    points_c1_unnormalized = np.dot(P1,X_w)
    points_c2_unnormalized = np.dot(P2,X_w)

    points_c1 = normalize_array(points_c1_unnormalized.T).T
    points_c2 = normalize_array(points_c2_unnormalized.T).T

    print("Points camera 1:")
    print(points_c1)

    print("Points camera 2:")
    print(points_c2)

    a_c1 = np.array([[points_c1[0][0], points_c1[1][0], points_c1[2][0]]]).T
    b_c1 = np.array([[points_c1[0][1], points_c1[1][1], points_c1[2][1]]]).T
    c_c1 = np.array([[points_c1[0][2], points_c1[1][2], points_c1[2][2]]]).T
    d_c1 = np.array([[points_c1[0][3], points_c1[1][3], points_c1[2][3]]]).T

    a_c2 = np.array([[points_c2[0][0], points_c2[1][0], points_c2[2][0]]]).T
    b_c2 = np.array([[points_c2[0][1], points_c2[1][1], points_c2[2][1]]]).T
    c_c2 = np.array([[points_c2[0][2], points_c2[1][2], points_c2[2][2]]]).T
    d_c2 = np.array([[points_c2[0][3], points_c2[1][3], points_c2[2][3]]]).T

    x1 = np.array([points_c1[0],points_c1[1]])

    plt.figure(1)
    plt.imshow(img1)
    plt.plot(x1[0, :], x1[1, :],'+r', markersize=15)
    plotLabeledImagePoints(x1, ['a','b','c','d','e'], 'r', (20,-20)) # For plotting with labels (choose one of the both options)
    #plotNumberedImagePoints(x1, 'r', (20,25)) # For plotting with numbers (choose one of the both options)
    plt.title('Image 1')
    plt.draw()  # We update the figure display
    print('Click in the image to continue...')
    plt.waitforbuttonpress()

    # #Exercise 2

    l_ab_image1 = np.cross(a_c1,b_c1, axis=0).reshape((3,))
    drawLine(np.array([[l_ab_image1[0]], [l_ab_image1[1]], [l_ab_image1[2]]]), 'g-', 1)

    l_cd_image1 = np.cross(c_c1,d_c1, axis=0).reshape((3,))
    drawLine(np.array([[l_cd_image1[0]], [l_cd_image1[1]], [l_cd_image1[2]]]), 'g-', 1)

    p1_l1_l2 = np.cross(l_ab_image1,l_cd_image1, axis=0).reshape((3,))
    p1_l1_l2 = p1_l1_l2 / p1_l1_l2[2]

    p_int1 = np.array([[p1_l1_l2[0]],[p1_l1_l2[1]], [1]])
    print("Intersection point image 1:")
    print(p_int1)

    plt.plot(p_int1[0], p_int1[1],'+r', markersize=15)
    plotLabeledImagePoints(p_int1, ['p_int'], 'r', (20,-20)) # For plotting with labels (choose one of the both options)

    plt.draw()
    print('Click in the image to continue...')
    plt.waitforbuttonpress()
    
    
    print('Close the figure to continue. Left button for orbit, right button for zoom.')
    plt.show()

    x2 = np.array([points_c2[0],points_c2[1]])

    plt.figure(2)
    plt.imshow(img2)
    plt.plot(x2[0, :], x2[1, :],'+r', markersize=15)
    plotLabeledImagePoints(x2, ['a','b','c','d','e'], 'r', (20,-20)) # For plotting with labels (choose one of the both options)
    #plotNumberedImagePoints(x1, 'r', (20,25)) # For plotting with numbers (choose one of the both options)
    plt.title('Image 2')
    plt.draw()  # We update the figure display
    print('Click in the image to continue...')
    plt.waitforbuttonpress()

    l_ab_image2 = np.cross(a_c2,b_c2, axis=0).reshape((3,))
    drawLine(np.array([[l_ab_image2[0]], [l_ab_image2[1]], [l_ab_image2[2]]]), 'g-', 1)

    l_cd_image2 = np.cross(c_c2,d_c2, axis=0).reshape((3,))
    drawLine(np.array([[l_cd_image2[0]], [l_cd_image2[1]], [l_cd_image2[2]]]), 'g-', 1)

    p2_l1_l2 = np.cross(l_ab_image2,l_cd_image2, axis=0)
    p2_l1_l2 = p2_l1_l2 / p2_l1_l2[2]

    p_int2 = np.array([[p2_l1_l2[0]],[p2_l1_l2[1]], [1]])
    print("Intersection point image 2:")
    print(p_int2)

    plt.plot(p_int2[0], p_int2[1],'+r', markersize=15)
    plotLabeledImagePoints(p_int2, ['p_int'], 'r', (20,-20)) # For plotting with labels (choose one of the both options)

    plt.draw()
    print('Click in the image to continue...')
    plt.waitforbuttonpress()

    AB_inf = X_B - X_A
    AB_inf = np.array([[AB_inf[0], AB_inf[1], AB_inf[2], 0]]).T

    print("AB line 3D infinite point:")
    print(AB_inf)

    #Projection of AB_inf in image
    ab_inf = np.dot(P1,AB_inf)
    ab_inf = ab_inf / ab_inf[2]
    print("ab line vanishing point:")
    print(ab_inf)

    #Exercise 3
    # Fit the least squares solution of inliers using svd
    X_plane = np.vstack((np.hstack((np.reshape(X_A,(3,1)), np.reshape(X_B,(3,1)), np.reshape(X_C,(3,1)), np.reshape(X_D,(3,1)))), np.ones((1, 4))))
    u, s, vh = np.linalg.svd(X_plane.T)
    l_ls = vh[-1, :]

    print("3D plane defined by A, B, C, D:")
    print(l_ls)

    distances = np.dot(X_w.T, l_ls) / np.sqrt(l_ls[0]**2 + l_ls[1]**2 + l_ls[2]**2)
    print("Distances of points A, B, C, D, E to plane:")
    print(distances)
    













