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



def draw_epipolar_line_img2(F):

    #Click point img 1
    fig = plt.figure(5)
    plt.imshow(img1, cmap='gray', vmin=0, vmax=255)
    plt.plot(x1[0, :], x1[1, :],'rx', markersize=10)
    plotNumberedImagePoints(x1, 'r', (10,0)) # For plotting with numbers (choose one of the both options)
    plt.title('Image 1')
    plt.draw()  # We update the figure display
    cid = fig.canvas.mpl_connect('button_press_event', lambda event: mouse_event_img1(event, F)) #compute line and plot in image 2

    print('Click in the image to continue...')
    plt.waitforbuttonpress()
    

def mouse_event_img1(event, F):

    clicked_x = np.array([[event.xdata], [event.ydata], [1]])
    print("Point clicked in image 1:")
    print(clicked_x)
    computeline_img2(clicked_x, F)

def computeline_img2(point, F):
    
    l = np.dot(F, point)
    print("Epipolar line:")
    print(l)

    fig = plt.figure(6)
    plt.imshow(img2, cmap='gray', vmin=0, vmax=255)
    plt.plot(x2[0, :], x2[1, :],'rx', markersize=10)
    plotNumberedImagePoints(x2, 'r', (10,0)) # For plotting with numbers (choose one of the both options)
    plt.title('Image 2')
    drawLine(l,'g',1)
    plt.draw()  # We update the figure display

    print('Click in the image to continue...')
    plt.waitforbuttonpress()

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


def plot_3D(X_computed, X_w, T_w_c1, T_w_c2):

    ##Plot the 3D cameras and the 3D points
    fig3D = plt.figure(4)

    ax = plt.axes(projection='3d', adjustable='box')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    drawRefSystem(ax, np.eye(4, 4), '-', 'W')
    drawRefSystem(ax, T_w_c1, '-', 'C1')
    drawRefSystem(ax, T_w_c2, '-', 'C2')

    ax.scatter(X_computed[0, :], X_computed[1, :], X_computed[2, :], marker='.', label = "estimated")
    #plotNumbered3DPoints(ax, X_computed, 'r', (0.1, 0.1, 0.1)) # For plotting with numbers (choose one of the both options)

    ax.scatter(X_w[0, :], X_w[1, :], X_w[2, :], marker='.', label = "real")
    #plotNumbered3DPoints(ax, X_w, 'b', (0.1, 0.1, 0.1)) # For plotting with numbers (choose one of the both options)

    ax.legend()

    #Matplotlib does not correctly manage the axis('equal')
    xFakeBoundingBox = np.linspace(0, 4, 2)
    yFakeBoundingBox = np.linspace(0, 4, 2)
    zFakeBoundingBox = np.linspace(0, 4, 2)
    plt.plot(xFakeBoundingBox, yFakeBoundingBox, zFakeBoundingBox, 'w.')
    print('Close the figure to continue. Left button for orbit, right button for zoom.')
    plt.show()

def plot_3D_sfm(X_computed, T_w_c1, T_w_c2):
    ##Plot the 3D cameras and the 3D points
    fig3D = plt.figure(4)

    ax = plt.axes(projection='3d', adjustable='box')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    drawRefSystem(ax, T_w_c1, '-', 'C1')
    drawRefSystem(ax, T_w_c2, '-', 'C2')

    ax.scatter(X_computed[0, :], X_computed[1, :], X_computed[2, :], marker='.', label = "estimated")
    #plotNumbered3DPoints(ax, X_computed, 'r', (0.1, 0.1, 0.1)) # For plotting with numbers (choose one of the both options)

    ax.legend()

    #Matplotlib does not correctly manage the axis('equal')
    xFakeBoundingBox = np.linspace(0, 4, 2)
    yFakeBoundingBox = np.linspace(0, 4, 2)
    zFakeBoundingBox = np.linspace(0, 4, 2)
    plt.plot(xFakeBoundingBox, yFakeBoundingBox, zFakeBoundingBox, 'w.')
    print('Close the figure to continue. Left button for orbit, right button for zoom.')
    plt.show()


def triangulate_3D(x1,x2,P1,P2):

    X_computed = np.zeros((4,len(X_w[0][:])))

    for i in range(n_matches):

        #print("Point: ", i)
        A = np.zeros((4,4))
        
        #Equations C1
        #First row
        A[0][0] = P1[2][0]*x1[0,i] - P1[0][0]
        A[0][1] = P1[2][1]*x1[0,i] - P1[0][1]
        A[0][2] = P1[2][2]*x1[0,i] - P1[0][2]
        A[0][3] = P1[2][3]*x1[0,i] - P1[0][3]
        #Second row
        A[1][0] = P1[2][0]*x1[1,i] - P1[1][0]
        A[1][1] = P1[2][1]*x1[1,i] - P1[1][1]
        A[1][2] = P1[2][2]*x1[1,i] - P1[1][2]
        A[1][3] = P1[2][3]*x1[1,i] - P1[1][3]

        #Equations C2

        A[2][0] = P2[2][0]*x2[0,i] - P2[0][0]
        A[2][1] = P2[2][1]*x2[0,i] - P2[0][1]
        A[2][2] = P2[2][2]*x2[0,i] - P2[0][2]
        A[2][3] = P2[2][3]*x2[0,i] - P2[0][3]
        #Second row
        A[3][0] = P2[2][0]*x2[1,i] - P2[1][0]
        A[3][1] = P2[2][1]*x2[1,i] - P2[1][1]
        A[3][2] = P2[2][2]*x2[1,i] - P2[1][2]
        A[3][3] = P2[2][3]*x2[1,i] - P2[1][3]

        u, s, vh = np.linalg.svd(A)
        X = np.reshape(vh[-1, :],(4,1))

        X = X / X[3][0]

        X_computed[0][i] = X[0][0]
        X_computed[1][i] = X[1][0]
        X_computed[2][i] = X[2][0]
        X_computed[3][i] = X[3][0]

        # print("3D point SVD:")
        # print(X)

    return X_computed

if __name__ == '__main__':
    np.set_printoptions(precision=4,linewidth=1024,suppress=True)

    # Load ground truth
    T_w_c1 = np.loadtxt('T_w_c1.txt')
    T_w_c2 = np.loadtxt('T_w_c2.txt')

    K_c = np.loadtxt('K_c.txt')
    X_w = np.loadtxt('X_w.txt')

    x1 = np.loadtxt('x1Data.txt')
    x2 = np.loadtxt('x2Data.txt')

    ## 2D plotting example
    img1 = cv2.cvtColor(cv2.imread('image1.png'), cv2.COLOR_BGR2RGB)
    img2 = cv2.cvtColor(cv2.imread('image2.png'), cv2.COLOR_BGR2RGB)


    # ##Plot the 3D cameras and the 3D points
    # fig3D = plt.figure(3)

    # ax = plt.axes(projection='3d', adjustable='box')
    # ax.set_xlabel('X')
    # ax.set_ylabel('Y')
    # ax.set_zlabel('Z')

    # drawRefSystem(ax, np.eye(4, 4), '-', 'W')
    # drawRefSystem(ax, T_w_c1, '-', 'C1')
    # drawRefSystem(ax, T_w_c2, '-', 'C2')

    # ax.scatter(X_w[0, :], X_w[1, :], X_w[2, :], marker='.')
    # plotNumbered3DPoints(ax, X_w, 'r', (0.1, 0.1, 0.1)) # For plotting with numbers (choose one of the both options)

    # #Matplotlib does not correctly manage the axis('equal')
    # xFakeBoundingBox = np.linspace(0, 4, 2)
    # yFakeBoundingBox = np.linspace(0, 4, 2)
    # zFakeBoundingBox = np.linspace(0, 4, 2)
    # plt.plot(xFakeBoundingBox, yFakeBoundingBox, zFakeBoundingBox, 'w.')
    # print('Close the figure to continue. Left button for orbit, right button for zoom.')
    # plt.show()


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

    #Exercise 1

    n_matches = x1.shape[1]

    # Find P matrices
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

    X_computed = triangulate_3D(x1, x2, P1, P2)
    plot_3D(X_computed, X_w, T_w_c1, T_w_c2)

    #Exercise 2

    #2.1
    F_provided = np.loadtxt('F_21_test.txt')
    print("F provided:")
    print(F_provided)

    draw_epipolar_line_img2(F_provided)

    #2.2
    T_c2_c1 = np.dot(T_c2_w,T_w_c1)
    print("T_c2_c1:")
    print(T_c2_c1)

    R_c2_c1 = T_c2_c1[0:3].T[0:3].T
    print("R_c2_c1")
    print(R_c2_c1)

    t_c2_c1 = T_c2_c1.T[-1][:]
    print("t_c2_c1:")
    print(t_c2_c1)

    #Compute matrix from t

    t = np.zeros((3,3))
    t[0][0] = 0
    t[0][1] = -t_c2_c1[2]
    t[0][2] = t_c2_c1[1]
    t[1][0] = t_c2_c1[2]
    t[1][1] = 0
    t[1][2] = -t_c2_c1[0]
    t[2][0] = -t_c2_c1[1]
    t[2][1] = t_c2_c1[0]
    t[2][2] = 0

    print("t matrix:")
    print(t)

    E = np.dot(t,R_c2_c1)
    print("Matrix E")
    print(E)

    #Compute matrix F

    F_computed = np.linalg.multi_dot([np.linalg.inv(K_c).T,E,np.linalg.inv(K_c)])

    print("Funtamental matrix F:")
    print(F_computed)

    #Draw lines with estimated F
    draw_epipolar_line_img2(F_computed)

    #2.3
    n_matches = x1.shape[1]
    A = np.zeros((n_matches, 9))
    for i in range(n_matches):

        x_0 = x1[0][i]
        x_1 = x2[0][i]
        y_0 = x1[1][i]
        y_1 = x2[1][i]
        w_0 = 1.0
        w_1 = 1.0

        A[i][0] = x_0*x_1
        A[i][1] = y_0*x_1
        A[i][2] = w_0*x_1
        A[i][3] = x_0*y_1
        A[i][4] = y_0*y_1
        A[i][5] = w_0*y_1
        A[i][6] = x_0*w_1
        A[i][7] = y_0*w_1
        A[i][8] = w_0*w_1

    u, s, vh = np.linalg.svd(A)
    F_matches_vector = np.reshape(vh[-1, :],(9,1))

    print("SVD result F components:")
    print(F_matches_vector)

    F_matches = np.zeros((3,3))
    F_matches[0][0] = F_matches_vector[0]
    F_matches[0][1] = F_matches_vector[1]
    F_matches[0][2] = F_matches_vector[2]
    F_matches[1][0] = F_matches_vector[3]
    F_matches[1][1] = F_matches_vector[4]
    F_matches[1][2] = F_matches_vector[5]
    F_matches[2][0] = F_matches_vector[6]
    F_matches[2][1] = F_matches_vector[7]
    F_matches[2][2] = F_matches_vector[8]

    print("F from matches:")
    print(F_matches)

    #Draw lines with estimated F from matches
    draw_epipolar_line_img2(F_matches)

    #2.4

    E = np.linalg.multi_dot([K_c.T,F_matches,K_c])
    print("Computed E from F:")
    print(E)

    u, s, vh = np.linalg.svd(E)
    t_estimated = np.reshape(u[-1,:], (3,1))
    t_estimated1 = u[-1,:]
    print("Estimated t:")
    print(t_estimated)

    W = np.zeros((3,3))
    W[0][0] = 0
    W[0][1] = -1
    W[0][2] = 0
    W[1][0] = 1
    W[1][1] = 0
    W[1][2] = 0
    W[2][0] = 0
    W[2][1] = 0
    W[2][2] = 1

    R_plus90_1 = np.linalg.multi_dot([u,W,vh.T])
    R_plus90_2 = - np.linalg.multi_dot([u,W,vh.T])
    R_minus90_1 = np.linalg.multi_dot([u,W.T,vh.T])
    R_minus90_2 = - np.linalg.multi_dot([u,W.T,vh.T])

    print("Four possible solutions:")
    print("R_90:")
    print(R_plus90_1)
    print("Determinant:")
    print(np.linalg.det(R_plus90_1))  #det=1
    print("-R_90:")
    print("Determinant:")
    print(np.linalg.det(R_plus90_2))
    print("R_-90:")
    print(R_minus90_1) 
    print("Determinant:")
    print(np.linalg.det(R_minus90_1)) #det=1
    print("-R_-90:")
    print(R_minus90_2)
    print("Determinant:")
    print(np.linalg.det(R_minus90_2))

    #Possible solutions R_plus90_1, R_minus90_1 (with +-t) -> 4 cases

    T_c2_c1 = ensamble_T(R_plus90_1, t_estimated1)
    T_w_c1 = np.dot(T_w_c2,T_c2_c1)
    #Triangulate points for 4 possible solutions
    P1_0 = np.dot(K_c, np.concatenate((np.eye(3), np.zeros((3,1))), axis=1))
    P2_0 = np.dot(K_c, np.concatenate((R_plus90_1, t_estimated), axis=1))
    X_computed = triangulate_3D(x1, x2, P1_0, P2_0)
    print("3D reconstructed sol 1:")
    print(X_computed)
    #plot_3D_sfm(X_computed, ensamble_T(R_plus90_1, t_estimated1), np.eye(4))
    plot_3D(X_computed, X_w, T_w_c1, T_w_c2)

    P1_1 = np.dot(K_c, np.concatenate((np.eye(3), np.zeros((3,1))), axis=1))
    P2_1 = np.dot(K_c, np.concatenate((R_plus90_1, -t_estimated), axis=1))
    X_computed = triangulate_3D(x1, x2, P1_1, P2_1)
    #plot_3D_sfm(X_computed, ensamble_T(R_plus90_1, -t_estimated1), np.eye(4))
    print("3D reconstructed sol 2:")
    print(X_computed)
    plot_3D(X_computed, X_w, T_w_c1, T_w_c2)

    T_c2_c1 = ensamble_T(R_minus90_1, t_estimated1)
    T_w_c1 = np.dot(T_w_c2,T_c2_c1)
    P1_2 = np.dot(K_c, np.concatenate((np.eye(3), np.zeros((3,1))), axis=1))
    P2_2 = np.dot(K_c, np.concatenate((R_minus90_1, t_estimated), axis=1))
    X_computed = triangulate_3D(x1, x2, P1_2, P2_2)
    #plot_3D_sfm(X_computed, ensamble_T(R_minus90_1, t_estimated1), np.eye(4))
    print("3D reconstructed sol 3:")
    print(X_computed)
    plot_3D(X_computed, X_w, T_w_c1, T_w_c2)

    P1_3 = np.dot(K_c, np.concatenate((np.eye(3), np.zeros((3,1))), axis=1))
    P2_3 = np.dot(K_c, np.concatenate((R_minus90_1, -t_estimated), axis=1))
    X_computed = triangulate_3D(x1, x2, P1_3, P2_3)
    #plot_3D_sfm(X_computed, ensamble_T(R_minus90_1, -t_estimated1), np.eye(4))
    print("3D reconstructed sol 4:")
    print(X_computed)
    plot_3D(X_computed, X_w, T_w_c1, T_w_c2)

    #Exercise 3









