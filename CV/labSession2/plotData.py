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

def draw_epipole_im2(e2):

    fig = plt.figure(6)
    plt.imshow(img2, cmap='gray', vmin=0, vmax=255)
    plt.plot(e2[0, :], e2[1, :],'rx', markersize=10)
    plt.title('Epipole Image 2')
    plt.draw()  # We update the figure display
    print('Close the figure to continue. Left button for orbit, right button for zoom.')
    plt.show()


def draw_epipolar_line_img2(F):

    #Click point img 1
    fig = plt.figure(5)
    plt.imshow(img1, cmap='gray', vmin=0, vmax=255)
    plt.plot(x1[0, :], x1[1, :],'rx', markersize=10)
    plotNumberedImagePoints(x1, 'r', (10,0)) # For plotting with numbers (choose one of the both options)
    plt.title('Image 1')
    plt.draw()  # We update the figure display
    cid = fig.canvas.mpl_connect('button_press_event', lambda event: mouse_event_img1(event, F)) #compute line and plot in image 2

    print('Close the figure to continue. Left button for orbit, right button for zoom.')
    plt.show()
    

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
    plt.plot(x1FloorData[0, :], x1FloorData[1, :],'rx', markersize=10,label = 'provided')
    plt.plot(x1HomographyFloorData[0, :], x1HomographyFloorData[1, :],'bx', markersize=10, label='homography')
    plt.legend()

    plotNumberedImagePoints(x1FloorData, 'r', (10,0)) # For plotting with numbers (choose one of the both options)
    plotNumberedImagePoints(x1HomographyFloorData, 'b', (10,0)) # For plotting with numbers (choose one of the both options)

    plt.title('Point transfer Image 1')
    plt.draw()  # We update the figure display
    print('Close the figure to continue. Left button for orbit, right button for zoom.')
    plt.show()

    fig = plt.figure(7)

    plt.imshow(img2, cmap='gray', vmin=0, vmax=255)
    plt.plot(x2HomographyFloorData[0, :], x2HomographyFloorData[1, :],'bx', markersize=10, label='homography')
    plt.plot(x2FloorData[0, :], x2FloorData[1, :],'rx', markersize=10, label = 'provided')
    plt.legend()

    plotNumberedImagePoints(x2FloorData, 'r', (10,0)) # For plotting with numbers (choose one of the both options)
    plotNumberedImagePoints(x2HomographyFloorData, 'b', (10,0)) # For plotting with numbers (choose one of the both options)

    plt.title('Point transfer Image 2')
    plt.draw()  # We update the figure display

    print('Close the figure to continue. Left button for orbit, right button for zoom.')
    plt.show()

    mean_error1 = 0.0
    mean_error2 = 0.0

    for i in range(x1FloorData.shape[1]):
        ex1 = x1FloorData[0][i] - x1HomographyFloorData[0][i]
        ey1 = x1FloorData[1][i] - x1HomographyFloorData[1][i]
        mean_error1 = mean_error1 + np.sqrt(ex1*ex1 + ey1*ey1)
        ex2 = x2FloorData[0][i] - x2HomographyFloorData[0][i]
        ey2 = x2FloorData[1][i] - x2HomographyFloorData[1][i]
        mean_error2 = mean_error2 + np.sqrt(ex2*ex2 + ey2*ey2)

    mean_error1 = mean_error1 / x1FloorData.shape[1]
    print("Mean error image 1:")
    print(mean_error1)
    mean_error2 = mean_error2 / x2FloorData.shape[1]
    print("Mean error image 2:")
    print(mean_error2)



def triangulate_3D(x1,x2,T_w_c1,T_w_c2):

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

    #Exercise 1

    X_computed = triangulate_3D(x1, x2, T_w_c1, T_w_c2)
    plot_3D(X_computed, X_w, T_w_c1, T_w_c2)

    #Exercise 2

    #2.1
    F_provided = np.loadtxt('F_21_test.txt')
    print("F provided:")
    print(F_provided)
    u, s, vh = np.linalg.svd(F_provided)
    e1 = np.reshape(vh[-1, :],(3,1))
    e1 = e1 / e1[2,:]
    print("Epipole 1 - optical centre of C2 in C1:")
    print(e1)
    u, s, vh = np.linalg.svd(F_provided.T)
    e2 = np.reshape(vh[-1, :],(3,1))
    e2 = e2 / e2[2,:]
    print("Epipole 2 - optical centre of C1 in C2:")
    print(e2)

    draw_epipolar_line_img2(F_provided)

    #2.2
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

    u, s, vh = np.linalg.svd(F_computed)
    e1 = np.reshape(vh[-1, :],(3,1))
    e1 = e1 / e1[2,:]
    print("Epipole 1 - optical centre of C2 in C1:")
    print(e1)
    u, s, vh = np.linalg.svd(F_computed.T)
    e2 = np.reshape(vh[-1, :],(3,1))
    e2 = e2 / e2[2,:]
    print("Epipole 2 - optical centre of C1 in C2:")
    print(e2)

    #Draw lines with estimated F
    draw_epipolar_line_img2(F_computed)

    draw_epipole_im2(e2)

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

    # E = np.linalg.multi_dot([K_c.T,F_matches,K_c])
    E = np.linalg.multi_dot([K_c.T,F_provided,K_c])

    print("Computed E from F:")
    print(E)

    u, s, vh = np.linalg.svd(E)

    t_estimated = u[:,2]
    print("Estimated t:")
    print(t_estimated)

    W = np.array([[0, -1, 0],
                [1, 0, 0],
                [0, 0, 1]])

    R_plus90_positive = np.linalg.multi_dot([u,W,vh.T])
    R_plus90_negative = - np.linalg.multi_dot([u,W,vh.T])
    R_minus90_positive = np.linalg.multi_dot([u,W.T,vh.T])
    R_minus90_negative = - np.linalg.multi_dot([u,W.T,vh.T])

    print("Four possible solutions:")
    print("R_90:")
    print(R_plus90_positive)
    print("Determinant:")
    print(np.linalg.det(R_plus90_positive))  #det=-1
    print("-R_90:")
    print("Determinant:")
    print(np.linalg.det(R_plus90_negative)) #det=1
    print("R_-90:")
    print(R_minus90_positive) 
    print("Determinant:")
    print(np.linalg.det(R_minus90_positive)) #det=-1
    print("-R_-90:")
    print(R_minus90_negative)
    print("Determinant:")
    print(np.linalg.det(R_minus90_negative)) #det=1

    # #Possible solutions -R_90, -R_-90 (with +-t) -> 4 cases
    
    # #Triangulate points for 4 possible solutions

    R1 = R_plus90_negative
    R2 = R_minus90_negative
    # # Four possible combinations of R and t
    poses = [(R1, t_estimated), (R1, -t_estimated), (R2, t_estimated), (R2, -t_estimated)]

    for idx, (R, t) in enumerate(poses):
        T_c2_to_c1 = np.eye(4)
        T_c2_to_c1[:3, :3] = R
        T_c2_to_c1[:3, 3] = t
        T_c2_to_c1[3, 3] = 1
        T_w_to_c1 = np.dot(T_w_c2, T_c2_to_c1)
        print("R and t solution")
        print(T_w_to_c1)

        X_computed = triangulate_3D(x1, x2, T_w_to_c1, T_w_c2)

        mean_error = 0.0
        for i in range(X_w.shape[1]):
                ex = X_w[0][i] - X_computed[0][i]
                ey = X_w[1][i] - X_computed[1][i]
                ez = X_w[2][i] - X_computed[2][i]
                mean_error = mean_error + np.sqrt(ex*ex + ey*ey + ez*ez)
        mean_error = mean_error / X_w.shape[1]

        print("Mean error:")
        print(mean_error)

        # Now, visualize the results for this solution
        fig3D = plt.figure(idx+6)

        ax = plt.axes(projection='3d', adjustable='box')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        drawRefSystem(ax, np.eye(4, 4), '-', 'W')
        drawRefSystem(ax, T_w_to_c1, '-', 'C1')
        drawRefSystem(ax, T_w_c2, '-', 'C2')

        ax.scatter(X_computed[0, :], X_computed[1, :], X_computed[2, :], marker='.', color = 'blue', label = 'triangulated')
        ax.scatter(X_w[0, :], X_w[1, :], X_w[2, :], color='red', marker='.', label = 'ground truth')
        ax.legend()
        #plotNumbered3DPoints(ax, X_w_calculated, 'r', (0.1, 0.1, 0.1)) # For plotting with numbers (choose one of the both options)

        #Matplotlib does not correctly manage the axis('equal')
        xFakeBoundingBox = np.linspace(0, 4, 2)
        yFakeBoundingBox = np.linspace(0, 4, 2)
        zFakeBoundingBox = np.linspace(0, 4, 2)
        plt.plot(xFakeBoundingBox, yFakeBoundingBox, zFakeBoundingBox, 'w.')
        plt.title(f'Solution {idx+1}')
        print('Close the figure to continue. Left button for orbit, right button for zoom.')
        plt.show()

    
    #Exercise 3

    #3.1 Homography from camera callibration

    Pi_1 = np.array([[0.0149,0.9483,0.3171,-1.7257]]).T
    n = Pi_1[0:3].T
    d = Pi_1[-1]
    t_c2_c1 = np.reshape(t_c2_c1, (3,1))

    A = R_c2_c1 - np.dot(t_c2_c1, n) / d

    H_2_1 = np.linalg.multi_dot([K_c, A, np.linalg.inv(K_c)])

    #3.2 Point transfer

    x1FloorData = np.loadtxt("x1FloorData.txt")
    x2FloorData = np.loadtxt("x2FloorData.txt")
    point_transfer(x1FloorData, x2FloorData, H_2_1)

    #3.3 Homography from matches
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
        w_0 = 1.0
        w_1 = 1.0

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











