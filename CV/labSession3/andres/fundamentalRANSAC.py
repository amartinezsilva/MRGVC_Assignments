#####################################################################################
#
# MRGCV Unizar - Computer vision - Laboratory 2
#
# Title: Line RANSAC fitting
#
# Date: 28 September 2020
#
#####################################################################################
#
# Authors: Jesus Bermudez, Richard Elvira, Jose Lamarca, JMM Montiel
#
# Version: 1.0
#
#####################################################################################

import cv2
import matplotlib.pyplot as plt
import numpy as np
import random
import scipy.linalg as scAlg
import math


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
    plt.plot(x1_inliers[0, :], x1_inliers[1, :],'rx', markersize=10)
    plotNumberedImagePoints(x1_inliers, 'r', (10,0)) # For plotting with numbers (choose one of the both options)
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
    plt.plot(x2_inliers[0, :], x2_inliers[1, :],'rx', markersize=10)
    plotNumberedImagePoints(x2_inliers, 'r', (10,0)) # For plotting with numbers (choose one of the both options)
    plt.title('Image 2')
    drawLine(l,'g',1)
    plt.draw()  # We update the figure display

    print('Click in the image to continue...')
    plt.waitforbuttonpress()

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


def calculate_RANSAC_attempts(P, inlier_rate, p):
    k = math.log(1 - P) / math.log(1 - (1 - inlier_rate)**p)
    #print(k)
    return int(k)

def RANSACFundamental(x1, x2):
        
    P = 0.9
    spurious_rate = 0.1
    inlier_rate = 1 - spurious_rate
    RANSACThreshold = 2
    RANSACminsetF = 8
    t = calculate_RANSAC_attempts(P, inlier_rate, RANSACminsetF)
    RANSAC_params = {'t': t, 'inlier_rate': inlier_rate, 'P': P, 'threshold': RANSACThreshold, 'minset':RANSACminsetF}
    
    keypoints0 = [cv2.KeyPoint(x=x, y=y, size=x1.shape[1]) for x, y in zip(x1[0], x1[1])]
    keypoints1 = [cv2.KeyPoint(x=x, y=y, size=x2.shape[1]) for x, y in zip(x2[0], x2[1])]
    img1 = cv2.cvtColor(cv2.imread('image1.png'), cv2.COLOR_BGR2RGB)
    img2 = cv2.cvtColor(cv2.imread('image2.png'), cv2.COLOR_BGR2RGB)

    N1 = normalizationMatrix(img1.shape[1], img1.shape[0])
    N2 = normalizationMatrix(img2.shape[1], img2.shape[0])

    # number m of random samples
    nVotesMin = 50
    nVotes = 0

    inliers_idx = []

    print('Total matches = ' + str(x1.shape[1]))


    while RANSAC_params['minset'] != RANSAC_params['t']:

        random_indices = np.random.choice(x1.shape[1], RANSACminsetF, replace=False)

        random_x1 = x1[:,random_indices]
        random_x2 = x2[:,random_indices] 

        #Normalize points before estimation
        random_x1 = N1 @ random_x1
        random_x2 = N2 @ random_x2

        #Compute Hypothesis
        F_model = calculateF(random_x1, random_x2)

        # Identify points not used for H calculation
        remaining_indices = [i for i in range(x1.shape[1]) if i not in random_indices]

        # Select the remaining points for evaluation
        x1_eval = x1[:, remaining_indices]
        x2_eval = x2[:, remaining_indices]

        for i in range(x1_eval.shape[1]):
            
            #Unnormalize F before evaluating matches
            F_model = N2.T @ F_model @ N1

            epipole_line_SVD = np.dot(F_model, x1_eval[:, i])
             # Calculate distance
            distance = abs(np.dot(epipole_line_SVD, x2_eval[:, i])) / np.linalg.norm(epipole_line_SVD[:2])
            if(distance < RANSAC_params['threshold']):
                nVotes+=1
                RANSAC_params['inlier_rate'] = nVotes / x1_eval.shape[1]
                RANSAC_params['t'] = calculate_RANSAC_attempts(RANSAC_params['P'], RANSAC_params['inlier_rate'], RANSAC_params['minset'])
                inliers_idx.append(i)
                print(RANSAC_params)

    if nVotes > nVotesMin:

        x1_inliers = x1[:,inliers_idx]
        x2_inliers = x2[:,inliers_idx]

        #Normalize points before estimation
        x1_inliers = N1 @ x1_inliers
        x2_inliers = N2 @ x2_inliers

        best_F = calculateF(x1_inliers, x2_inliers)

        #Unnormalize F before evaluating matches
        best_F = N2.T @ best_F @ N1

        inliers_list = []

        for idx in inliers_idx:
            match = cv2.DMatch(_queryIdx=idx, _trainIdx=idx, _distance=0)
            inliers_list.append(match)

        # Visualize the matches producing the hypothesis and the inliers
        plt.title('Best inliers')
        visualize_matches(img1, keypoints0, img2, keypoints1, inliers_list)

        return best_F, x1_inliers, x2_inliers
    
def normalizationMatrix(nx,ny):
    """
 
    -input:
        nx: number of columns of the matrix
        ny: number of rows of the matrix
    -output:
        Nv: normalization matrix such that xN = Nv @ x
    """
    Nv = np.array([[1/nx, 0, -1/2], [0, 1/ny, -1/2], [0, 0, 1]])
    return Nv

def calculateF(x1,x2):

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

    # print("SVD result F components:")
    # print(F_matches_vector)

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

    # print("F from matches:")
    # print(F_matches)
    
    return F_matches


def visualize_matches(img1, kp1, img2, kp2, matches):
    img_matched = cv2.drawMatches(img1, kp1, img2, kp2, matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    plt.imshow(img_matched)
    plt.show()

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


if __name__ == '__main__':
    # np.set_printoptions(precision=4,linewidth=1024,suppress=True)
    
    ## 2D plotting example
    img1 = cv2.cvtColor(cv2.imread('image1.png'), cv2.COLOR_BGR2RGB)
    img2 = cv2.cvtColor(cv2.imread('image2.png'), cv2.COLOR_BGR2RGB)
    
    # x1_sift = np.loadtxt('x1_sift.txt')
    # x1_sift = np.vstack([x1_sift, np.ones((1,x1_sift.shape[1]))]) #need to be in homogeneous coordinates 

    # x2_sift = np.loadtxt('x2_sift.txt')
    # x2_sift = np.vstack([x2_sift, np.ones((1,x2_sift.shape[1]))]) #need to be in homogeneous coordinates 

    # # #RANSAC + SIFT
    # F_sift = RANSACFundamental(x1_sift, x2_sift)
    # print("Automatic F using Superglue:")
    # print(F_sift)

    path = './superglue_results/image1_image2_matches.npz'
    npz = np.load(path)
    #npz.files    
    x1_superglue = npz['keypoints0'].T
    x1_superglue = np.vstack([x1_superglue, np.ones((1,x1_superglue.shape[1]))]) #need to be in homogeneous coordinates 
    
    x2_superglue = npz['keypoints1'].T
    x2_superglue = np.vstack([x2_superglue, np.ones((1,x2_superglue.shape[1]))]) #need to be in homogeneous coordinates 
    
    #RANSAC + SuperGlue
    F_superglue, x1_inliers, x2_inliers = RANSACFundamental(x1_superglue, x2_superglue)
    print("Automatic F using Superglue:")
    print(F_superglue)
    print("Number of inliers:")
    print(x1_inliers.shape[1])

    draw_epipolar_line_img2(F_superglue)

